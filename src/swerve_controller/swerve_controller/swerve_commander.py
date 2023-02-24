import sys
from typing import Dict

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Float64MultiArray

from .swerve_joints import (
    JointEnd,
    JointKey,
    JointKind,
    JointSide,
    joint_name_from_key,
    joint_positions,
    joints,
)


class SwerveCommander(Node):
    """
    Swerve commander implements the kinematics of a swerve drive robot.
    It computes the math to determine the wheel angles and velocities fron
    robot twist.

    Subscribers:
    "/cmd_vel" - Twist

    Publishers:
    4 steering (position) controllers and 4 wheel (velocity) controllers
    "/{steering|wheel}_{front|rear}_{right|left}/commands" - Float64MultiArray
    """

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
        # init node superclass
        super().__init__("swerve_commander")

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

        # set up a publisher for each of the 8 swerve joints using key iterator
        self.joint_publisher: Dict[JointKey, Publisher] = {}
        for key in joints():
            topic: str = f"/{joint_name_from_key(key)}/commands"
            self.joint_publisher[key] = self.create_publisher(
                Float64MultiArray, topic, 10
            )

        # register cmd_vel subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.twist_to_swerve, 10
        )

    def twist_to_swerve(self, msg: Twist) -> None:
        """
        generates steering angle and wheel velocity for 8 swerve joints
        from input twist message on /cmd_vel and publishes it on respective
        controller topics.
        """
        vx: float = msg.linear.x
        vy: float = msg.linear.y
        wz: float = msg.angular.z

        # To prevent wheels from changing direction when stopping
        # if (wz == 0 and vx == 0 and vy == 0):
        # wheel position remains same
        if np.allclose([0, 0, 0], [wz, vx, vy]):
            self.stop()
            return

        # iterate over wheels
        # * calculate linear velocity components in cartesian plane
        # * calculate joint angle and direction given limited joint angles
        # * publish steering angle and wheel velocity for each wheel
        for end, side in joint_positions():
            # calculate linear velocity components in cartesian plane
            # for individual wheels
            kx = 1.0 if side is JointSide.RIGHT else -1.0
            ky = 1.0 if end is JointEnd.FRONT else -1.0
            wheel_vel_x = vx + (kx * self.wheelbase * wz / 2)
            wheel_vel_y = vy + (ky * self.wheel_track * wz / 2)

            # limit steering angle to +/- 90 degrees
            steering_angle: float = np.arctan2(wheel_vel_y, wheel_vel_x)
            rot_dir: float = 1.0

            if steering_angle > np.pi / 2.0:
                steering_angle -= np.pi
                rot_dir *= -1.0

            elif steering_angle < -np.pi / 2.0:
                steering_angle += np.pi
                rot_dir *= -1.0

            # calculate angular velocity of wheel
            wheel_angular_velocity = (
                rot_dir
                * np.sqrt(wheel_vel_x**2 + wheel_vel_y**2)
                / self.wheel_radius
            )

            # publish steering angle and wheel velocity
            for kind, value in (
                (JointKind.STEERING, steering_angle),
                (JointKind.WHEEL, wheel_angular_velocity),
            ):
                joint_msg: Float64MultiArray = Float64MultiArray()
                joint_msg.data = [value]
                key: JointKey = (kind, end, side)
                self.joint_publisher[key].publish(joint_msg)

    def stop(self):
        """
        set angular velocity of all 4 wheel positions to 0.0
        """
        msg: Float64MultiArray = Float64MultiArray()
        msg.data = [0.0]
        for end, side in joint_positions():
            key: JointKey = (JointKind.WHEEL, end, side)
            self.joint_publisher[key].publish(msg)


def main():
    rclpy.init(args=sys.argv)

    swerve_commander = SwerveCommander()
    rclpy.spin(swerve_commander)


if __name__ == "__main__":
    main()
