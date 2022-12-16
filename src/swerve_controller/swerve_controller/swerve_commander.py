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
    swerve commander implements the output path for a swerve drive controller

    listens on

        /cmd_vel

    for twist messages describing the desired
    trajectory for the entire vehicle and translates that into
    steering angles and wheel velocities for the eight joints of a
    swerve drive system and publishes them on the eight swerve drive topics:

        /{steering|wheel}_{front|rear}_{right|left}/commands

    the steering angles and wheel velocities for the joints are sent as
    float64 multi arrays because that is the message type expected by
    ros2_control joint controllers
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
    max_motor_speed_descriptor = ParameterDescriptor(
        name="max_motor_speed",
        type=ParameterType.PARAMETER_DOUBLE,
        description="maximum motor speed in rpm",
    )

    # Rotation per minute to Radians per second
    RPM_TO_RADPS = 0.10472

    def __init__(self):
        super().__init__("swerve_commander")  # init node superclass with name

        # declare and get node parameters
        self.wheelbase: float
        self.wheel_track: float
        self.wheel_radius: float
        self.max_motor_speed: float
        for descriptor in (
            self.wheelbase_descriptor,
            self.wheel_track_descriptor,
            self.wheel_radius_descriptor,
            self.max_motor_speed_descriptor,
        ):
            self.declare_parameter(name=descriptor.name, descriptor=descriptor)
            value = self.get_parameter(descriptor.name).get_parameter_value()
            setattr(self, descriptor.name, value.double_value)

        # distance to wheel pivot from robot center
        self.r: float = (
            np.sqrt(self.wheelbase**2 + self.wheel_track**2) / 2.0
        )

        # x & y components of r
        self.rx = self.r * np.cos(np.arctan2(self.wheelbase, self.wheel_track))
        self.ry = self.r * np.sin(np.arctan2(self.wheelbase, self.wheel_track))

        # set up a publisher for each of the 8 swerve joints using key iterator

        self.joint_publisher: Dict[JointKey, Publisher] = {}

        for key in joints():
            topic: str = f"/{joint_name_from_key(key)}/commands"
            self.joint_publisher[key] = self.create_publisher(
                Float64MultiArray, topic, 10
            )

        # register cmd_vel topic subscription callback
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.twist_to_swerve, 10
        )

    def check_motor_saturation(
        self, swerve_state: Dict[JointKey, float]
    ) -> Dict[JointKey, float]:
        """
        Check commanded speed with max speed motor is capable of.
        Even if one of the commanded speeds is greater then all
        speeds need to be scaled down.
        """

        # Reset max wheel velocity
        max_wheel_vel = 0
        saturated_swerve_state: Dict[JointKey, float] = {}

        for end, side in joint_positions():
            kind = JointKind.wheel
            if np.fabs(swerve_state[(kind, end, side)]) > max_wheel_vel:
                max_wheel_vel = np.fabs(swerve_state[(kind, end, side)])

        if max_wheel_vel > self.max_motor_speed * self.RPM_TO_RADPS:
            scaling_factor = (
                self.max_motor_speed * self.RPM_TO_RADPS / max_wheel_vel
            )

            for kind, end, side in joints():
                key: JointKey = (kind, end, side)
                if kind == JointKind.wheel:
                    saturated_swerve_state[key] = (
                        swerve_state[key] * scaling_factor
                    )
                else:
                    saturated_swerve_state[key] = swerve_state[key]

            return saturated_swerve_state

        else:
            return swerve_state

    def publish_controller_commands(
        self, swerve_cmds: Dict[JointKey, float]
    ) -> None:
        """
        Publish steering angle and wheel velocity commands on respective topics
        """

        for kind, end, side in joints():
            joint_msg: Float64MultiArray = Float64MultiArray()
            key: JointKey = (kind, end, side)
            joint_msg.data = [swerve_cmds[key]]
            self.joint_publisher[key].publish(joint_msg)

    def stop(self):
        """
        set angular velocity of all 4 wheel positions to 0.0
        """
        msg: Float64MultiArray = Float64MultiArray()
        msg.data = [0.0]
        for end, side in joint_positions():
            key: JointKey = (JointKind.wheel, end, side)
            self.joint_publisher[key].publish(msg)

    def twist_to_swerve(self, msg: Twist) -> None:
        """
        generates steering angle and wheel velocity for 8 swerve joints
        from input twist message on /cmd_vel
        """
        vx: float = msg.linear.x
        vy: float = msg.linear.y
        wz: float = msg.angular.z

        # Dict to store joint command values
        swerve_commands: Dict[JointKey, float] = {}

        # To prevent wheels from changing direction when braking
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
            kx = 1.0 if side is JointSide.right else -1.0
            ky = 1.0 if end is JointEnd.front else -1.0
            wheel_vel_x = vx + (kx * self.rx * wz)
            wheel_vel_y = vy + (ky * self.ry * wz)

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

            for kind, value in [
                (JointKind.steering, steering_angle),
                (JointKind.wheel, wheel_angular_velocity),
            ]:

                swerve_commands[(kind, end, side)] = value

        saturation_checked_cmds = self.check_motor_saturation(swerve_commands)
        self.publish_controller_commands(saturation_checked_cmds)


def main():
    rclpy.init(args=sys.argv)

    swerve_commander = SwerveCommander()
    rclpy.spin(swerve_commander)


if __name__ == "__main__":
    main()
