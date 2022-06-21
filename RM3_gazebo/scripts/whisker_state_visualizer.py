#!/usr/bin/python3
"""
Node to subscribe to WhiskerStates and publish the end-tip locations as
a point cloud map.

@author: Walid Remmas
@contact: walid.remmas@taltech.ee
@date: 09-06-2022
"""

import rclpy

from rclpy.node import Node
from robominer_msgs.msg import WhiskerArray, Whisker
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry

import os
from ament_index_python.packages import get_package_share_directory
import yaml

import numpy as np

######### Order of whisker rows and columns #########
"""
       ***************************************
     *(7,7)(7,6)(7,5)(7,4)(7,3)(7,1)(7,1)(7,0)*
       *********** front of robot ************
       ######################################
*(5,7) #  (3,7)    (2,7)    (1,7)    (0,7)  # (4,7)*
*(5,6) #  (3,6)    (2,6)    (1,6)    (0,6)  # (4,6)*
*(5,5) #  (3,5)    (2,5)    (1,5)    (0,5)  # (4,5)*
*(5,4) #  (3,4)    (2,4)    (1,4)    (0,4)  # (4,4)*
*(5,3) #  (3,3)    (2,3)    (1,3)    (0,3)  # (4,3)*
*(5,2) #  (3,2)    (2,2)    (1,2)    (0,2)  # (4,2)*
*(5,1) #  (3,1)    (2,1)    (1,1)    (0,1)  # (4,1)*
*(5,0) #  (3,0)    (2,0)    (1,0)    (0,0)  # (4,0)*
       ######################################
       ************ rear of robot ************
     *(6,7)(6,6)(6,5)(6,4)(6,3)(6,2)(6,1)(6,0)*
       ***************************************
"""
#####################################################


class WhiskerVisualizer(Node):
    """Class that helps to publish and visualize the whisker states as point clouds in RViz
    """

    def __init__(self):
        super().__init__('whisker_state_visualizer')

        # Load simulation parameters
        # --------------------------------------------
        simulation_parameters_yaml = os.path.join(
                get_package_share_directory('rm3_gazebo'),
                'config',
                'simulation_parameters.yaml'
                )
        with open(simulation_parameters_yaml, 'r') as file:
            sim_params = yaml.load(file, Loader=yaml.FullLoader)
        # --------------------------------------------

        self.mapping_type = sim_params['sensors']['whiskers']['map_type']
        self.whiskers_num = sim_params['sensors']['whiskers']['whiskers_num']
        self.whisker_pc_pub = self.create_publisher(PointCloud, '/WhikserPointCloud', 10)

        self.pose = None
        self.gotOdom = False

        ## Whikser parameters
        # -------------------------------------------------------------------
        self.bottom_whisker_length = sim_params['sensors']['whiskers']['bottom_whisker_length']
        self.side_whisker_length = sim_params['sensors']['whiskers']['side_whisker_length']
        self.side_array_angle = sim_params['sensors']['whiskers']['side_array_angle']
        self.whisker_array_angle = np.pi -  2.15  # inclination of sides whiskers in rads

        # pos of side whiskers biases in robot_frame
        whisker_array_x = 0.405
        whisker_array_y = 0.35
        whisker_array_z = 0.33


        self.whisker_x = -0.0345
        self.whisker_x_bias = 0.075
        # bottom whiskers:
        self.whisker_y = 0.001
        self.whisker_z = 0.02

        self.array_location = {0: [0, -0.20, 0.285], 1: [0, -0.12, 0.285],
                               2: [0,  -0.04, 0.285], 3: [0,  0.04, 0.285],
                               4: [0,  0.12, 0.285], 5: [0,  0.20, 0.285],
                               6: [0, -whisker_array_y, whisker_array_z],
                               7: [0,  whisker_array_y, whisker_array_z],
                               8: [-whisker_array_x,  0, whisker_array_z],
                               9: [ whisker_array_x,  0, whisker_array_z]}

        # an orientation multiplication is needed for side arrays
        rotation_x = self.rotateHomogeneous('x', -self.whisker_array_angle)
        rotation_y = self.rotateHomogeneous('y', self.whisker_array_angle)
        rotation_z = self.rotateHomogeneous('z', np.pi/2)

        self.array_orinetation = {6: self.rotateHomogeneous('x', -self.whisker_array_angle),
                                  7: self.rotateHomogeneous('x', self.whisker_array_angle),
                                  8: self.rotateHomogeneous('y', self.whisker_array_angle),
                                  9: self.rotateHomogeneous('y', -self.whisker_array_angle)}
        # -------------------------------------------------------------------
        self.create_subscription(WhiskerArray, '/WhiskerStates', self.WhiskerStateCallback, 10)
        self.create_subscription(Odometry, '/odom/unfiltered', self.OdomCallback, 10)

        self.whiskers_viz_timer_period = 0.1
        self.pointCloud_timer = self.create_timer(self.whiskers_viz_timer_period, self.processWhiskerData)
        self.whiskerData = []
        self.gotWhiskers = False

    def OdomCallback(self, msg):
        if not self.gotOdom:
            self.gotOdom = True

    def WhiskerStateCallback(self, msg):
        self.whiskerData = msg
        if not self.gotWhiskers:
            self.gotWhiskers = True

    def processWhiskerData(self):
        # Process the whiskers data only when an odom message is received.
        if self.gotOdom and self.gotWhiskers:
            pc = PointCloud()
            pc.header.frame_id = "chassis"
            msg = self.whiskerData
            for i in range(self.whiskers_num):
                point = Point32()

                whisker = msg.whiskers[i]

                condition1 = self.mapping_type == "Always"
                condition2 = whisker.x**2 + whisker.y**2 > 0.02
                if condition1 or condition2:
                    # This applies for bottom whiskers
                    if whisker.pos.row_num < 6:
                        whisker_pos = np.array([self.whisker_x + (whisker.pos.col_num-3)*self.whisker_x_bias,
                                        self.whisker_y,
                                        -self.whisker_z])
                        whisker_pos += np.array(self.array_location[whisker.pos.row_num])

                        tip_position = [self.bottom_whisker_length * np.sin(whisker.x),
                                        self.bottom_whisker_length * np.sin(whisker.y)*np.cos(whisker.x),
                                        -self.bottom_whisker_length * np.cos(whisker.x)*np.cos(whisker.y)]

                        whisker_tip = whisker_pos + tip_position
                        point.x = whisker_tip[0]
                        point.y = whisker_tip[1]
                        point.z = whisker_tip[2]
                        pc.points.append(point)

                    # This applies for left and right whiskers
                    elif whisker.pos.row_num < 8:
                        whisker_pos = np.zeros(4)
                        whisker_tip = np.zeros(4)

                        whisker_pos = np.array([self.whisker_x + (whisker.pos.col_num-3)*self.whisker_x_bias,
                                        self.whisker_y,
                                        -self.whisker_z])

                        whisker_pos += np.array(self.array_location[whisker.pos.row_num])

                        tip_position = [self.side_whisker_length * np.sin(whisker.x),
                                        self.side_whisker_length * np.sin(whisker.y)*np.cos(whisker.x),
                                        -self.side_whisker_length * np.cos(whisker.x)*np.cos(whisker.y)]

                        whisker_tip_hom = np.array([tip_position[0], tip_position[1], tip_position[2], 1.0])

                        whisker_tip_transformed = np.dot(self.translateHomogeneous(whisker_pos),
                                                         np.dot(self.array_orinetation[whisker.pos.row_num], whisker_tip_hom))

                        point.x = whisker_tip_transformed[0]
                        point.y = whisker_tip_transformed[1]
                        point.z = whisker_tip_transformed[2]
                        pc.points.append(point)

                    # This applies for rear and front whiskers arrays
                    # elif whisker.pos.row_num < 8:
                    #     whisker_pos = np.zeros(4)
                    #     whisker_tip = np.zeros(4)
                    #
                    #     whisker_pos = np.array([self.whisker_y,
                    #                     self.whisker_x + (whisker.pos.col_num-3)*self.whisker_x_bias,
                    #                     -self.whisker_z])
                    #
                    #     whisker_pos += np.array(self.array_location[whisker.pos.row_num])
                    #
                    #     tip_position = [-self.side_whisker_length * np.sin(whisker.y)*np.cos(whisker.x),
                    #                     self.side_whisker_length * np.sin(whisker.x),
                    #                     -self.side_whisker_length * np.cos(whisker.x)*np.cos(whisker.y)]
                    #
                    #     whisker_tip_hom = np.array([tip_position[0], tip_position[1], tip_position[2], 1.0])
                    #
                    #     whisker_tip_transformed = np.dot(self.translateHomogeneous(whisker_pos),
                    #                                      np.dot(self.array_orinetation[whisker.pos.row_num], whisker_tip_hom))
                    #
                    #     point.x = whisker_tip_transformed[0]
                    #     point.y = whisker_tip_transformed[1]
                    #     point.z = whisker_tip_transformed[2]
                    #     pc.points.append(point)

            pc.header.stamp = self.get_clock().now().to_msg()
            self.whisker_pc_pub.publish(pc)

    def translateHomogeneous(self, translation):
        """
        Function to translate a point in space
        @input: translation -- (3x1) vector of translation [x,y,z]
        @return: Homogeneous translation matrix
        """
        R = np.array([[1.0, 0.0, 0.0, translation[0]], \
                      [0.0, 1.0, 0.0, translation[1]],\
                      [0.0, 0.0, 1.0, translation[2]], \
                      [0.0, 0.0, 0.0, 1.0]])

        return R


    def rotateHomogeneous(self, axis, angle):
        """
        Function to rotate a point in space
        @input: axis -- name of axis of rotation "x", "y" or "z"
        @input: angle --  amount of rotation in rads
        @return: Homogeneous rotation matrix
        """
        R = np.eye(4)
        a = angle
        if axis == 'x':
            R = np.array([[1,0,0,0], \
                    [0, np.cos(a), -np.sin(a), 0],\
                    [0, np.sin(a), np.cos(a), 0], \
                    [0, 0, 0, 1]])
        elif axis == 'y':
            R = np.array([[np.cos(a),0,np.sin(a),0], \
                    [0, 1, 0, 0],\
                    [-np.sin(a), 0, np.cos(a), 0], \
                    [0, 0, 0, 1]])

        elif axis == 'z':
            R = np.array([[np.cos(a), -np.sin(a), 0,0], \
                    [np.sin(a), np.cos(a), 0, 0],\
                    [0, 0, 1, 0], \
                    [0, 0, 0, 1]])

        return R


def main(args=None):
    rclpy.init(args=args)
    whisker_node = WhiskerVisualizer()
    rclpy.spin(whisker_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    whisker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
