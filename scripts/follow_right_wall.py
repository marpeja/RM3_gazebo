#!/usr/bin/python3

import rclpy
import math
from rclpy.node import Node
from rclpy import qos
import numpy as np

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FollowWall(Node):

	def __init__(self):
		super().__init__('follow_wall')
		self.subscription = self.create_subscription(
			LaserScan,
			'/scan',
			self.scan_listener_callback,
			qos.qos_profile_sensor_data)
		self.publisher = self.create_publisher(
			Twist,
			'/cmd_vel_keyboard',
			10)
		self.subscription  # prevent unused variable warning
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.timer = self.create_timer(1.0, self.on_timer)
		self.to_frame_rel = 'world'
		self.from_frame_rel = 'base_link'
		self.yaw_angle = 0.0
		self.p_translation = [0.0, 0.0, 0.0]
		self.p_rotation = [0.0, 0.0, 0.0, 0.0]
		self.go_back = False
		self.turn_back = False
		self.stop = 0
		self.stuck = 0
		self.angle_backwards = 0.0
		self.store_angle_backwards = True
		self.angular_recovery = 0.0
		self.is_origin = False
		self.state_ = 0
		self.state_dict_ = {
							0: 'find the wall',
							1: 'turn left',
							2: 'follow the wall',
							}
	
	def on_timer(self):

		try:
			now = rclpy.time.Time()
			trans = self.tf_buffer.lookup_transform(
			self.to_frame_rel,
			self.from_frame_rel,
			now)
		except TransformException as ex:
			self.get_logger().info(
			f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
			return
		
		translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
		rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
		[roll, pitch, yaw] = self.euler_from_quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
		self.yaw_angle = yaw
		
		has_moved = self.check_difference(self.p_translation, self.p_rotation, translation, rotation)
		#self.get_logger().info('Previous translation: {} and previous rotation: {}'.format(self.p_translation, self.p_rotation))
		#self.get_logger().info('Actual translation: {} and actual rotation: {}'.format(translation, rotation))
		#self.get_logger().info('HAS MOVED: {}'.format(has_moved))
		self.p_translation = translation
		self.p_rotation = rotation
		
		
		if self.turn_back == True:
			yaw_angle_difference = self.check_difference_yaw_angle()
			self.get_logger().info('Initial angle {}, Actual angle{}, difference {}'.format(self.angle_backwards, self.yaw_angle, yaw_angle_difference))
			if self.store_angle_backwards == True:
				self.angle_backwards = self.yaw_angle
				self.store_angle_backwards = False
			if yaw_angle_difference > 3.10:
				self.get_logger().info('Difference {} and finish'.format(yaw_angle_difference))
				self.turn_back = False
				self.store_angle_backwards = True
		elif has_moved == False:		
			self.stop = self.stop + 1			
			if self.stop > 4:
				self.go_back = True
				self.stop = 0
				
		elif has_moved == True and self.go_back == True:
			self.get_logger().info(f'Going backwards')
			self.stuck = self.stuck +1
			if self.stuck > 2:
				self.go_back = False
				self.stuck = 0
				
		elif has_moved == True:
			self.stop = 0     #Restart the count
			self.check_origin(translation[0], translation[1])
	
	def check_difference(self, p_translation, p_rotation, translation, rotation):
		difference = False
		
		for x in range (0, len(p_translation)):
			if abs(p_translation[x] - translation[x]) > 0.01:
				difference = True
		
		for x in range (0, len(p_rotation)):
			if abs(p_rotation[x] - rotation[x]) > 0.007:
				difference = True
		
		return difference
		
	def check_difference_yaw_angle(self):
		
		if self.angle_backwards > 0 and self.yaw_angle < 0:
			return (6.28 - self.angle_backwards + self.yaw_angle)
		else:
			return abs(self.angle_backwards - self.yaw_angle)
		
	def check_origin(self, x, y):
		radius = math.sqrt(x*x+y*y)
		
		if radius < 1.1 and abs(self.yaw_angle) > 1.6:
			self.is_origin = True
		
	def euler_from_quaternion(self, x, y, z, w):
		"""
		Convert a quaternion into euler angles (roll, pitch, yaw)
		roll is rotation around x in radians (counterclockwise)
		pitch is rotation around y in radians (counterclockwise)
		yaw is rotation around z in radians (counterclockwise)
		"""
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll_x = math.atan2(t0, t1)

		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch_y = math.asin(t2)
 
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw_z = math.atan2(t3, t4)

		return roll_x, pitch_y, yaw_z # in radians
	
	def change_state(self, state):
		if state is not self.state_:
		    #self.get_logger().info('Wall follower - {} - {}'.format(state, self.state_dict_[state]))
		    self.state_ = state
		return self.move()
		
	def move(self):
		linear_x = 0.0
		linear_y = 0.0
		angular_z = 0.0

		if self.state_ == 0:
			linear_x = 2.0
			linear_y = 0.0
			angular_z = -2.0

		elif self.state_ == 1:
			linear_x = 0.0
			linear_y = 0.0
			angular_z = 3.0
		
		elif self.state_ == 2:
			linear_x = 3.0
			linear_y = 0.0
			angular_z = 0.0

		return [linear_x, linear_y, angular_z]

	def scan_listener_callback(self, msg):
	
		#self.get_logger().info('Data: {}'.format(msg.ranges[0:638]))
		
		fright = list(filter(lambda v: v==v, msg.ranges[0:212]))
		front = list(filter(lambda v: v==v, msg.ranges[213:425]))
		fleft = list(filter(lambda v: v==v, msg.ranges[426:638]))
		
		if len(fright) == 0:
			fright.append(10)
			
		if len(front) == 0:
			front.append(10)
			
		if len(fleft) == 0:
			fleft.append(10)
		
		regions = {
			'fright': min(min(fright[0:len(fright)]), 10),
			'front': min(min(front[0:len(front)]), 10),
			'fleft': min(min(fleft[0:len(fleft)]), 10),
		}
		
		if regions['fright'] == float('-inf'):
			regions['fright'] = 0.4
		if regions['front'] == float('-inf'):
			regions['front'] = 0.4
		if regions['fleft'] == float('-inf'):
			regions['fleft'] = 0.4			

		#self.get_logger().info('Min_Fleft: {}'.format(regions['fright']))
		self.get_logger().info('Data: {}'.format(regions))
		
		self.take_action(regions)
		

	def take_action(self, regions):
		msg = Twist()
		linear_x = 0.0
		linear_y = 0.0
		angular_z = 0.0

		state_description = ''
		
		d = 1.6
		d_min = 0.9
		
		if self.is_origin == True:
			state_description = 'case end - Back to origin. End of track'
			linear_x = 0.0
			linear_y = 0.0
			angular_z = 0.0
			
		elif self.turn_back == False:
			if regions['fright'] > 9.8 and regions['front'] > 9.8 and regions['fleft'] > 9.8:
				state_description = 'case 0 - turn back'
				self.turn_back = True
			elif regions['fright'] > d and regions['front'] > d and regions['fleft'] > d:
				state_description = 'case 1 - nothing -> find the wall'
				[linear_x, linear_y, angular_z] = self.change_state(0)
				
			elif regions['fright'] > d and regions['front'] > d and regions['fleft'] < d:
				state_description = 'case 2 - fleft -> find the wall'
				[linear_x, linear_y, angular_z] = self.change_state(0)
				
			elif regions['fright'] > d and regions['front'] < d and regions['fleft'] > d:
				state_description = 'case 3 - front -> find the wall'
				[linear_x, linear_y, angular_z] = self.change(0)
				
			elif regions['fright'] < d and regions['front'] > d and regions['fleft'] > d:
				if regions['fright'] > d_min:
					state_description = 'case 4 - fright -> follow the wall'
					[linear_x, linear_y, angular_z] = self.change_state(2)
				else:
					state_description = 'case 4 - fright -> turn left'
					[linear_x, linear_y, angular_z] = self.change_state(1)
				
			elif regions['fright'] < d and regions['front'] < d and regions['fleft'] > d:
				state_description = 'case 5 - fright and front - > turn left'
				[linear_x, linear_y, angular_z] = self.change_state(1)
				
			elif regions['fright'] < d and regions['front'] > d and regions['fleft'] < d:
				if regions['fright'] < d_min:
					state_description = 'case 6 - fright and fleft -> turn left'
					[linear_x, linear_y, angular_z] = self.change_state(1)
				elif regions['fleft'] < d_min:
					state_description = 'case 6 - fright and fleft -> find the wall'
					[linear_x, linear_y, angular_z] = self.change_state(0)
				else:
					state_description = 'case 6 - fright and fleft -> follow the wall'
					[linear_x, linear_y, angular_z] = self.change_state(2)
				
			elif regions['fright'] > d and regions['front'] < d and regions['fleft'] < d:
				state_description = 'case 7 - front and fleft -> find the wall'
				[linear_x, linear_y, angular_z] = self.change_state(0)
				
			elif regions['fright'] < d and regions['front'] < d and regions['fleft'] < d:
				if regions['fright'] < d_min or regions['front'] < d_min:
					state_description = 'case 8 - fright front and fleft -> turn left'
					[linear_x, linear_y, angular_z] = self.change_state(1)
				elif regions['fleft'] < d_min:
					state_description = 'case 8 - fright front and fleft -> find the wall'
					[linear_x, linear_y, angular_z] = self.change_state(0)
				else:
					state_description = 'case 8 - fright front and fleft -> follow the wall'
					[linear_x, linear_y, angular_z] = self.change_state(2)
								
			if self.go_back == True:
				state_description = 'case 9 - going back'
				if self.stuck == 0:
					self.angular_recovery = angular_z
				if linear_x == 0.0:
					linear_x = 1.0
				linear_x = -linear_x
				linear_y = -linear_y
				angular_z = self.angular_recovery		
		else:
			linear_x = 0.0
			linear_y = 0.0
			angular_z = 3.0
			
			if self.go_back == True:
				state_description = 'case 9 - going back'
				linear_x = -1.5
				linear_y = 0.0
				angular_z = 3.0
			
		self.get_logger().info(state_description)
		msg.linear.x = linear_x
		msg.linear.y = linear_y
		msg.angular.z = angular_z
		self.publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)

	follow_wall = FollowWall()

	rclpy.spin(follow_wall)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
	follow_wall.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
