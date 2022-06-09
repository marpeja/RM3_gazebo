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

import numpy as np

class WhiskerPublisher(Node):
    """Docstring

    more docstring
    """

    def __init__(self):
        super().__init__('whisker_state_publisher')

        self.create_subscription(JointState, '/joint_states', self.JointStateCallback, 10)

        self.whisker_pub = self.create_publisher(WhiskerArray, '/WhiskerStates', 10)
        self.whisker_total = 64


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
            all_whisker_msg.representation = "Cartesian"

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
                #print(whisker_msg)
                if col_num == 8:
                    row_num += 1
                    col_num = 0

            self.whisker_pub.publish(all_whisker_msg)
            
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
