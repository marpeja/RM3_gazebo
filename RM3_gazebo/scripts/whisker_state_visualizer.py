#!/usr/bin/python3
"""
Node to subscribe to WhiskerStates and publish the end-tip location as
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
    """Docstring

    more docstring
    """

    def __init__(self):
        super().__init__('whisker_state_visualizer')

        self.create_subscription(WhiskerArray, '/WhiskerStates', self.WhiskerStateCallback, 10)
        self.create_subscription(Odometry, '/odom/unfiltered', self.OdomCallback, 10)

        self.whisker_pc_pub = self.create_publisher(PointCloud, '/WhikserPointCloud', 10)
        self.pose = None
        self.gotOdom = False

        ## Whikser parameters
        # -------------------------------------------------------------------
        self.whisker_length = 0.15
        self.whisker_array_angle = 2.15 - np.pi/2 # inclination of sides whiskers in rads

        # pos of side whiskers biases in robot_frame
        whisker_array_x = 0.39
        whisker_array_y = 0.33
        whisker_array_z = 0.32


        self.whisker_x = -0.0345
        self.whisker_x_bias = 0.075
        # bottom whiskers:
        self.whisker_y = 0.001
        self.whisker_z = 0.02
        # side whiskers:
        self.whisker_array_coeff = 0.032

        self.array_location = {0: [0, -0.16, 0.275], 1: [0, -0.05, 0.275],
                               2: [0,  0.05, 0.275], 3: [0,  0.16, 0.275],
                               4: [0, -whisker_array_y, whisker_array_z],
                               5: [0,  whisker_array_y, whisker_array_z],
                               6: [-whisker_array_x,  0, whisker_array_z],
                               7: [ whisker_array_x,  0, whisker_array_z]}

        # an orientation multiplication is needed for side arrays
        rotation_x = self.rotateHomogeneous('x', self.whisker_array_angle)
        rotation_y = self.rotateHomogeneous('y', self.whisker_array_angle)
        rotation_z = self.rotateHomogeneous('z', np.pi/2)

        self.array_orinetation = {4: rotation_x,
                                  5: -rotation_x,
                                  6: np.dot(rotation_y, rotation_z),
                                  7: np.dot(-rotation_y, rotation_z)}
        self.viz_thershold = 0.05
        # -------------------------------------------------------------------


    def OdomCallback(self, msg):
        self.pose = msg.pose.pose.position
        self.gotOdom = True

    def WhiskerStateCallback(self, msg):

        if self.gotOdom:
            pc = PointCloud()
            pc.header.stamp = self.get_clock().now().to_msg()
            pc.header.frame_id = "base_link"
            for i in range(len(msg.whiskers)):
                point = Point32()

                whisker = msg.whiskers[i]

                # This applies for bottom whiskers
                if whisker.pos.row_num < 4:
                    whisker_pos = np.array([self.whisker_x + (whisker.pos.col_num-3)*self.whisker_x_bias,
                                    self.whisker_y,
                                    -self.whisker_z])
                    whisker_pos += np.array(self.array_location[whisker.pos.row_num])

                    tip_position = [self.whisker_length * np.cos(whisker.y)*np.sin(whisker.x),
                                    self.whisker_length * np.sin(whisker.y)*np.sin(whisker.x),
                                    -self.whisker_length * np.cos(whisker.x)]

                    whisker_tip = whisker_pos + tip_position
                    point.x = whisker_tip[0]
                    point.y = whisker_tip[1]
                    point.z = whisker_tip[2]
                    pc.points.append(point)

                """
                # This applies for left and right arrays
                elif whisker.pos.row_num == 4:
                    whisker_pos = np.array([self.whisker_x + (whisker.pos.col_num-3)*self.whisker_x_bias,
                                    -self.whisker_y,
                                    self.whisker_z])

                    # whisker_pos = np.dot(self.array_orinetation[whisker.pos.row_num][0:3,0:3], whisker_pos)
                    whisker_pos += np.array(self.array_location[whisker.pos.row_num])
                    whisker_pos += [0.0, -self.whisker_array_coeff * np.sin(self.whisker_array_angle), -self.whisker_array_coeff * np.cos(self.whisker_array_angle)]

                    tip_position = [self.whisker_length * np.cos(whisker.y)*np.sin(whisker.x),
                                      -self.whisker_length * (np.sin(whisker.y)*np.sin(whisker.x) +np.cos(self.whisker_array_angle) ),
                                      -self.whisker_length * (np.cos(-whisker.x)-np.sin(self.whisker_array_angle))]
                    # whisker_pos += np.dot(self.array_orinetation[whisker.pos.row_num][0:3,0:3], whisker_pos)

                    #
                    # tip_position = np.dot(self.array_orinetation[whisker.pos.row_num][0:3,0:3], tip_position)

                    whisker_tip = whisker_pos + tip_position

                    # # print("whisker_pos:", whisker_pos)
                    # # print("tip_position:", tip_position)
                    # whisker_tip = whisker_pos + tip_position

                    # print(whisker_tip)
                    point.x = whisker_tip[0]
                    point.y = whisker_tip[1]
                    point.z = whisker_tip[2]
                    pc.points.append(point)
                """
            self.whisker_pc_pub.publish(pc)


    def rotateHomogeneous(self, axis, angle):
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
