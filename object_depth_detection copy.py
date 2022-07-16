from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from math_functions import *

import numpy as np
import rospy
import math

def InitListeners():
	global point_publisher

	rospy.init_node('object_depth', anonymous=True)
	rospy.Subscriber("/jetson_inference/detection", Point, find_point_depth)
	rospy.Subscriber("/d400/depth/image_rect_raw", Image, update_depth_image)
	rospy.Subscriber("/t265/odom/sample", Odometry, update_position)

	point_publisher = rospy.Publisher('object_depth/detection', Point, queue_size=5)

	rospy.spin()

def find_point_depth(point):
	global img
	
	x_pixel_ratio = 3.2 / 1.0
	y_pixel_ratio = 2.8 / 1.0

	depth_image_x = RoundTo((point.x / x_pixel_ratio), 1) + 120
	depth_image_y = RoundTo((point.y / y_pixel_ratio), 1) + 90

	#min_y = RoundTo((point.y / y_pixel_ratio), 1) + 90

	if img.shape[0] != 1 and point.z == 15:
		found_depth = img[depth_image_y][depth_image_x]
		find_point_position(depth_image_x, found_depth)

def find_point_position(point_x, depth):
	global robot_position
	global robot_orientation
	global point_publisher

	heading = (320 - point_x) * ((29 * (math.pi / 180)) / 320)
	true_heading = robot_orientation.z + heading
	to_publish = Point()
	to_publish.x = robot_position.x + (depth / 1000) * math.cos(true_heading)
	to_publish.y = robot_position.y + (depth / 1000) * math.sin(true_heading)
	to_publish.z = 0
	
	point_publisher.publish(to_publish)

def update_depth_image(image):
	global img

	img = np.frombuffer(image.data, dtype='uint16')
	img.resize((image.height, image.width))

def update_position(odometry):
	'''
	Updates global robot position variable
	'''
	global robot_position
	global robot_orientation

	robot_position = odometry.pose.pose.position
	robot_orientation = QuaternionToEuler(odometry.pose.pose.orientation)

if __name__ == "__main__":
	global img

	img = np.zeros((1,1), dtype='uint16')
	InitListeners()
