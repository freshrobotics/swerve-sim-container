import sys
from dataclasses import dataclass
from queue import Empty
from queue import SimpleQueue as Queue
from typing import Final, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from .swerve_joints import (
    JointEnd,
    JointKind,
    JointSide,
    joint_name_from_key,
    joint_positions,
)


@dataclass
class SwerveJointState:
    stamp: Time
    steering_position: Tuple[float, float, float, float]
    wheel_velocity: Tuple[float, float, float, float]


class SwerveOdometer(Node):
    """
    implements odometry for a swerve drive robot

    Subscribers:
        /joint_states -> sensor_msgs.msg.JointState

      for steering position or wheel velocity (from encoder ticks) for all
      eight swerve joints

    Publishers:
        /odom/wheel -> nav_msgs.msg.Odometry

      and

        /tf -> geometry_msgs.msg.TransformStamped
    """

    PARENT_FRAME: Final[str] = "odom"
    BASE_FRAME: Final[str] = "base_link"
    WHEEL_ODOM_TOPIC: Final[str] = "/odom/wheel"
    JOINT_STATES_TOPIC: Final[str] = "/joint_states"
    ODOM_PUB_FREQ: Final[float] = 30.0
    NUM_ACTUATORS: Final[int] = 8
    NUM_STATE: Final[int] = 3  # vx, vy, vyaw

    # descriptors for node parameters
    wheelbase_descriptor = ParameterDescriptor(
        name="wheelbase",
        type=ParameterType.PARAMETER_DOUBLE,
        description="distance between front and rear wheels in meters",
    )
    wheel_track_descriptor = ParameterDescriptor(
        name="wheel_track",
        type=ParameterType.PARAMETER_DOUBLE,
        description="distance between left and right wheels in meters",
    )
    wheel_radius_descriptor = ParameterDescriptor(
        name="wheel_radius",
        type=ParameterType.PARAMETER_DOUBLE,
        description="radius of wheel in meters",
    )

    def __init__(self):
        super().__init__("swerve_odometer")  # init node superclass with name

        # declare and get node parameters
        self.wheelbase: float
        self.wheel_track: float
        self.wheel_radius: float
        for descriptor in (
            self.wheelbase_descriptor,
            self.wheel_track_descriptor,
            self.wheel_radius_descriptor,
        ):
            self.declare_parameter(name=descriptor.name, descriptor=descriptor)
            value = self.get_parameter(descriptor.name).get_parameter_value()
            setattr(self, descriptor.name, value.double_value)

        # generate matrix for Twist calculation
        # swerve drive has 8 actuators (4 steering + 4 rotation)
        # equations are expressed in matrix form as A[8x3] * X[3x1] = B[8x1]
        # Refer to swerve drive kinematics

        # set up 'A' matrix
        self.mat_a = np.zeros((self.NUM_ACTUATORS, self.NUM_STATE))
        for i, (end, side) in enumerate(joint_positions()):
            kx = 1.0 if side is JointSide.RIGHT else -1.0
            ky = 1.0 if end is JointEnd.FRONT else -1.0
            self.mat_a[2 * i] = [1, 0, kx * self.wheelbase / 2]
            self.mat_a[2 * i + 1] = [0, 1, ky * self.wheel_track / 2]

        # setup queue to hold pending reported joint states (filled by joint
        # state handler) and variable to hold last published joint state
        # (to calculate delta)
        self.state_q: Queue[SwerveJointState] = Queue()
        self.last_joint_state: Optional[SwerveJointState] = None

        # initial pose is at origin (with orientation w set to 1.0)
        self.pose: Pose = Pose()
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = 0.0
        self.pose.orientation.w = 1.0

        # setup joint states callback
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.JOINT_STATES_TOPIC,
            self.handle_joint_states,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

        # set up wheel odom publisher & transform broadcaster
        self.wheel_odom_pub = self.create_publisher(
            Odometry, self.WHEEL_ODOM_TOPIC, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub_timer = self.create_timer(
            1.0 / self.ODOM_PUB_FREQ,
            self.publish_odometry,
            callback_group=ReentrantCallbackGroup(),
        )

    def handle_joint_states(self, msg: JointState) -> None:
        """
        pushes new swerve joint state onto queue

        - callback for joint states subscriber
        - finds joint values by name and gathers them into `SwerveJointState`
        """

        # attempt to gather state of 8 named swerve joints from message
        steering_position: List[float] = []
        wheel_velocity: List[float] = []
        for kind, gathered, reported in (
            (JointKind.STEERING, steering_position, msg.position),
            (JointKind.WHEEL, wheel_velocity, msg.velocity),
        ):
            for end, side in joint_positions():
                name: str = f"{joint_name_from_key((kind, end, side))}_joint"
                index: int = msg.name.index(name)
                if index < 0:  # if any joint names are missing give up
                    self.get_logger().debug(f"joint '{name}' not in msg")
                    return
                gathered.append(reported[index])

        # we have state of all 8 swerve joints - push it onto state queue
        self.state_q.put(
            SwerveJointState(
                stamp=msg.header.stamp,
                steering_position=(  # explicitly list for mypy
                    steering_position[0],
                    steering_position[1],
                    steering_position[2],
                    steering_position[3],
                ),
                wheel_velocity=(  # explicitly list for mypy
                    wheel_velocity[0],
                    wheel_velocity[1],
                    wheel_velocity[2],
                    wheel_velocity[3],
                ),
            )
        )

    def publish_odometry(self) -> None:
        """
        called by odom pub timer to publish odom based on joint state updates
        """

        # check queue for updates to swerve joint state

        # if there have been no updates to swerve joint state just return
        waiting: int = self.state_q.qsize()
        if waiting < 1:
            return

        if waiting > 1:
            self.get_logger().debug(f"skipping {waiting - 1} joint states")

        # if there is no last state initialize with first state off of queue
        if self.last_joint_state is None:
            try:
                self.last_joint_state = self.state_q.get_nowait()
            except Empty:
                self.get_logger().warn("joint state queue unexpectedly empty!")
                return
            waiting -= 1
            if waiting < 1:
                return

        # set most recent waiting joint state off of queue as next state
        next_joint_state: Optional[SwerveJointState] = None
        while waiting > 0:
            try:
                next_joint_state = self.state_q.get_nowait()
                waiting -= 1
            except Empty:
                break

        if next_joint_state is None:
            self.get_logger().warn("joint state queue unexpectedly empty!")
            return

        # calculate current twist based on new joint state
        current_twist: Twist = self.compute_twist(next_joint_state)

        # calculate time delta between joint states
        time_delta_s = (
            Time.from_msg(next_joint_state.stamp)
            - Time.from_msg(self.last_joint_state.stamp)
        ).nanoseconds * 10**-9  # convert to seconds

        # update position and last joint state
        self.update_pose(current_twist, time_delta_s)
        self.last_joint_state = next_joint_state

        # publish updated odom msg
        now: Time = next_joint_state.stamp

        odom_msg = Odometry()
        odom_msg.header.frame_id = self.PARENT_FRAME
        odom_msg.child_frame_id = self.BASE_FRAME
        odom_msg.header.stamp = now
        odom_msg.twist.twist = current_twist
        odom_msg.pose.pose = self.pose
        self.wheel_odom_pub.publish(odom_msg)

        # publish odom transform
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = self.PARENT_FRAME
        odom_tf.child_frame_id = self.BASE_FRAME
        odom_tf.header.stamp = now
        odom_tf.transform.translation.x = self.pose.position.x
        odom_tf.transform.translation.y = self.pose.position.y
        odom_tf.transform.rotation.x = self.pose.orientation.x
        odom_tf.transform.rotation.y = self.pose.orientation.y
        odom_tf.transform.rotation.z = self.pose.orientation.z
        odom_tf.transform.rotation.w = self.pose.orientation.w
        self.tf_broadcaster.sendTransform(odom_tf)

    def compute_twist(self, joint_state: SwerveJointState) -> Twist:
        """
        Returns robot twist computed from joint_state

        - called within publish_odometry() function
        - computes velocities using inverse kinematics of swerve drive
        - uses QR decomposition for LSE solution of the overdetermined system
        """

        wheel_velocity_components: List[float] = []
        for steering_position, wheel_velocity in zip(
            joint_state.steering_position, joint_state.wheel_velocity
        ):
            if wheel_velocity < 0:
                wheel_velocity *= -1.0
                steering_position += np.pi

            linear_velocity = wheel_velocity * self.wheel_radius
            wheel_velocity_components.append(
                linear_velocity * np.cos(steering_position)
            )  # x
            wheel_velocity_components.append(
                linear_velocity * np.sin(steering_position)
            )  # y

        # Set up 'B' Matrix
        mat_b = np.array(wheel_velocity_components).T

        # qr decomposition solution
        mat_q, mat_r = np.linalg.qr(self.mat_a)
        mat_qtb = np.dot(mat_q.T, mat_b)
        velocity: Twist = Twist()
        (
            velocity.linear.x,
            velocity.linear.y,
            velocity.angular.z,
        ) = np.linalg.solve(mat_r, mat_qtb)

        return velocity

    def update_pose(self, velocity: Twist, time_delta: int) -> None:
        """
        Updates the robot pose (x, y, yaw) from it's current position
        to next by integrating the twist velocity over a time interval.

        Units:
        Twist.linear velocities - m/s
        Twist.angular velocities - rad/s
        time_delta - seconds
        pose.position - m
        pose.orientation - quaternion
        """

        yaw = euler_from_quaternion(
            [
                self.pose.orientation.x,
                self.pose.orientation.y,
                self.pose.orientation.z,
                self.pose.orientation.w,
            ]
        )[2]
        self.pose.position.x += (
            velocity.linear.x * np.cos(yaw) - velocity.linear.y * np.sin(yaw)
        ) * float(time_delta)
        self.pose.position.y += (
            velocity.linear.x * np.sin(yaw) + velocity.linear.y * np.cos(yaw)
        ) * float(time_delta)
        yaw += velocity.angular.z * float(time_delta)
        (
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w,
        ) = quaternion_from_euler(0.0, 0.0, yaw)


def main():
    rclpy.init(args=sys.argv)

    swerve_odometer = SwerveOdometer()
    rclpy.spin(swerve_odometer)


if __name__ == "__main__":
    main()
