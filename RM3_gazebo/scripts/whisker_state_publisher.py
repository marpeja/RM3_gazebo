#!/usr/bin/python3
"""
Node to subscribe to JointStateBroadcaster from Gazebo, and publish the whisker states
as a WhiskerArray.msg
@author: Walid Remmas
@contact: walid.remmas@taltech.ee
@date: 09-06-2022
"""

import rclpy

from rclpy.node import Node
from robominer_msgs.msg import WhiskerArray, Whisker
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point32, TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster


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


class WhiskerPublisher(Node):
    """Class that publishes the states of whisker angles.
    """

    def __init__(self):
        super().__init__('whisker_state_publisher')

        self.whisker_pub = self.create_publisher(WhiskerArray, '/WhiskerStates', 10)
        self.whisker_total = 64
        self.declare_parameter('which_representation')
        self.which_representation = self.get_parameter('which_representation').value
        self.whisker_length = 0.15

        self.br = TransformBroadcaster(self)
        self.gotOdom = False

        self.create_subscription(Odometry, '/odom/unfiltered', self.OdomCallback, 10)
        self.create_subscription(JointState, '/joint_states', self.JointStateCallback, 10)


    def JointStateCallback(self, msg):
        # making sure the jointState message contains all joints (128 for whiskers + 4 screws)
        if len(msg.name) == self.whisker_total * 2 + 4:
            whisker_msg = Whisker()
            all_whisker_msg = WhiskerArray()

            # To always ensure that the joints values are correct, they have to be sorted
            # by alphabetical order (knowing that bottom_right_right should be first)

            # use dictionary to have both names and values
            whiskers_data = {k: v for k, v in zip(msg.name, np.round(msg.position, 3))}
            # sort by names
            sorted_whiskers_data = dict( sorted(whiskers_data.items(), key=lambda x: x[0].lower()) )

            # get sorted names and positions in lists
            whiskers_names = list(sorted_whiskers_data.keys())
            whiskers_positions = list(sorted_whiskers_data.values())

            all_whisker_msg.header.stamp = self.get_clock().now().to_msg()
            all_whisker_msg.num_mux = 6
            all_whisker_msg.num_sensors = 8
            all_whisker_msg.representation = self.which_representation

            row_num = 0
            col_num = 0
            for i in range(0, len(whiskers_positions)-4, 2):
                whisker_msg = Whisker()
                whisker_msg.pos.row_num = row_num
                whisker_msg.pos.col_num = col_num
                whisker_msg.pos.offset_y = 0.0
                whisker_msg.has_error = False

                # get angles from joints (sorted by names in alphabetical order)
                whisker_msg.x = whiskers_positions[i]
                whisker_msg.y = whiskers_positions[i+1]
                whisker_msg.z = 0.0 # not used atm
                all_whisker_msg.whiskers.append(whisker_msg)
                col_num += 1
                if col_num == 8:
                    row_num += 1
                    col_num = 0

            self.whisker_pub.publish(all_whisker_msg)

    def OdomCallback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z

        t.transform.rotation.x = orientation.x
        t.transform.rotation.y = orientation.y
        t.transform.rotation.z = orientation.z
        t.transform.rotation.w = orientation.w

        # Send the transformation
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    whisker_node = WhiskerPublisher()
    rclpy.spin(whisker_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    whisker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
