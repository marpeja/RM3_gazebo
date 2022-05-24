#!/usr/bin/python3
"""
Takes motor module velocities as input from robot software.
Calculates robot forward kinematics to determine body velocity vector.
Publishes cmd_vel.

@author: Roza Gkliva
@contact: roza.gkliva@ttu.ee
@date: 15-12-2021
"""

import rclpy

from rclpy.node import Node
from robominer_msgs.msg import MotorModuleCommand
from geometry_msgs.msg import Twist

import numpy as np
from math import pi, tan


class MotorToBodyVel(Node):
    """Docstring

    more docstring
    """

    def __init__(self):
        super().__init__('motor_to_body_vel')

        self.screw_radius = 0.078  # m
        self.screw_helix_angle = pi/6 # pi/6 for fl and rr screws, -pi/6 for fr and rl
        self.lx = 0.15
        self.ly = 0.3
        self.lin_speed_multiplier = 1
        self.ang_speed_multiplier = 1

        self.screw_speeds = [0.0, 0.0, 0.0, 0.0]
        self.fr_vel = 0.0
        self.rr_vel = 0.0
        self.rl_vel = 0.0
        self.fl_vel = 0.0
        self.rpm_to_radpersec = 2*pi/60.0

        self.fwd_kinematics = 1.0/4.0 * np.array([
            [-tan(self.screw_helix_angle), -tan(self.screw_helix_angle), tan(self.screw_helix_angle), tan(self.screw_helix_angle)],
            [-1.0,  1.0, 1.0,-1.0],
            [-1/(self.lx + 1/tan(self.screw_helix_angle) * self.ly), -1/(self.lx + 1/tan(self.screw_helix_angle) * self.ly), -1/(self.lx + 1/tan(self.screw_helix_angle) * self.ly), -1/(self.lx + 1/tan(self.screw_helix_angle) * self.ly)]
        ])

        self.create_subscription(MotorModuleCommand, '/motor0/motor_rpm_setpoint', self.front_right, 10)
        self.create_subscription(MotorModuleCommand, '/motor1/motor_rpm_setpoint', self.rear_right, 10)
        self.create_subscription(MotorModuleCommand, '/motor2/motor_rpm_setpoint', self.rear_left, 10)
        self.create_subscription(MotorModuleCommand, '/motor3/motor_rpm_setpoint', self.front_left, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.kinematics_timer_period = 0.1  # seconds
        self.kinematics_timer = self.create_timer(self.kinematics_timer_period, self.motor_to_body_vel)

    def front_right(self, msg):
        self.fr_vel = msg.motor_rpm_goal
        # self.get_logger().info(f'front_right: {self.fr_vel}')

    def rear_right(self, msg):
        self.rr_vel = msg.motor_rpm_goal
        # self.get_logger().info(f'rear_right: {self.rr_vel}')

    def rear_left(self, msg):
        self.rl_vel = msg.motor_rpm_goal
        # self.get_logger().info(f'rear_left: {self.rl_vel}')

    def front_left(self, msg):
        self.fl_vel = msg.motor_rpm_goal
        # self.get_logger().info(f'front_left: {self.fl_vel}')

    def motor_to_body_vel(self):
        body_vel = Twist()

        self.screw_speeds = np.array([self.fr_vel, self.rr_vel, self.rl_vel, self.fl_vel]) * self.rpm_to_radpersec

        self.robot_twist = self.screw_radius * np.dot(self.fwd_kinematics, self.screw_speeds) 

        body_vel.linear.x = self.robot_twist[0] * self.lin_speed_multiplier 
        body_vel.linear.y = self.robot_twist[1] * self.lin_speed_multiplier
        body_vel.angular.z = self.robot_twist[2] * self.ang_speed_multiplier

        self.cmd_vel_pub.publish(body_vel)


def main(args=None):
    rclpy.init(args=args)
    motor_to_body_vel = MotorToBodyVel()
    rclpy.spin(motor_to_body_vel)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_to_body_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
