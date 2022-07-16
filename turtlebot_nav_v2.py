#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#from urllib import robotparser
import rospy
import math

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from math_functions import *

import sys, select, termios, tty

motor_publisher=0

robot_position = [0.0, 0.0]
robot_orientation = [0.0, 0.0, 0.0]

robot_linear_vel = 0.0
robot_angular_vel = 0.0

occupancy_map = np.zeros(1)
occupancy_resolution = 1.0
occupancy_metadata = ""

target_point = [0.0, 0.0]
robot_path = [[0.0,0.0], [(2**2)/4, (2**2)/4], [1 * (2**2), -(2**2)/2], [1 * (2**2) + 0.5, -(2**2)/2], [1*(2**2) + 0.5, -(2**2)/2 + 0.5]]

def InitListeners():	
    global motor_publisher

    rospy.Subscriber("occupancy", OccupancyGrid, UpdateMap)
    # rospy.Subscriber('odom', Odometry, UpdateOdom) # robot sensor
    motor_publisher = rospy.Publisher('~cmd_vel', Twist, queue_size=5)
    rospy.Subscriber('/t265/odom/sample', Odometry, UpdateOdom) #https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
    rospy.spin()

def UpdateOdom(odometry):
    UpdatePose(odometry.pose)
    UpdateTwist(odometry.twist)

    global robot_position
    global robot_path
    global motor_publisher

    control_speed = robot_linear_vel
    control_turn = robot_angular_vel
    
    target_pos = robot_path[0]
    robot_heading = robot_orientation[2]

    # print(str(target_pos) + " " + str(robot_position) + " " + str(target_heading) + " " + str(robot_heading))

    if((abs(robot_position[0] - target_pos[0]) < 0.05) and (abs(robot_position[1] - target_pos[1]) < 0.05)):
        if(len(robot_path) == 1):
            print("done")
        else:
            robot_path = robot_path[1:]

    if(len(robot_path) > 0):
        turn_sign = 1
        target_heading = (math.tan((target_pos[1]-robot_position[1])/(target_pos[0]-robot_position[0])))
        if(abs(target_heading - robot_heading) > 0.1):
            target_speed = 0
            target_turn = 0.5
            if (target_heading - robot_heading > 0):
                turn_sign = 1
            else:
                turn_sign = -1
            status = "turning"
        else:
            target_speed = 0.2
            target_turn = 0
            turn_sign = 1
            status = "driving"
        
        print(str(target_pos) + " " + str(robot_position) + " " + str(target_heading) + " " + str(robot_heading)) 
        if target_speed > control_speed:
            control_speed = min( target_speed, control_speed + 0.1 )
        elif target_speed < control_speed:
            control_speed = max( target_speed, control_speed - 0.1 )
        else:
            control_speed = target_speed

        if target_turn > control_turn:
            control_turn = min( abs(target_turn), abs(control_turn) + 0.2 )
        elif target_turn < control_turn:
            control_turn = max( abs(target_turn), abs(control_turn) - 0.2 )
        else:
            control_turn = abs(target_turn)

        control_turn = control_turn * turn_sign

        print(str(control_speed) + " " + str(control_turn) + "  " + str(target_speed) + " " + str(target_turn))
        # if status == "turning" and (control_speed) != 0:
        #     control_turn = 0
        # elif status == "driving" and (control_turn) != 0:
        #     control_speed = 0
    else:
        control_speed = 0
        control_turn = 0

    twist = Twist()
    twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
    print(twist)
    motor_publisher.publish(twist)

def UpdatePose(pose):
    global robot_position
    global robot_orientation

    position = pose.pose.position
    robot_position = [position.x, position.y]

    orientation = pose.pose.orientation
    orientation_euler = QuaternionToEuler(orientation)
    print(orientation_euler)
    robot_orientation = orientation_euler

def UpdateTwist(twist):
    global robot_linear_vel
    global robot_angular_vel

    robot_linear_vel = twist.twist.linear.x
    robot_angular_vel = twist.twist.angular.z

def UpdateMap(occupancy_grid):
    global occupancy_map
    global occupancy_resolution
    global occupancy_metadata

    map = np.array(occupancy_grid.data)
    occupancy_map = ProcessMap(map)

    occupancy_resolution = occupancy_grid.info.resolution
    occupancy_metadata = occupancy_grid.info

def ProcessMap(map):
    imageArray = parse_occupancy_map(map).astype("uint8")
	kernel = np.ones((2,2), np.uint8)
	erodedArray = cv2.erode(imageArray, kernel, iterations=1)
	return erodedArray

if __name__=="__main__":
    global robot_position
    global robot_orientation
    global robot_linear_vel
    global robot_angular_vel
    global robot_path 

    robot_position = [0.0, 0.0]
    robot_orientation = [0.0, 0.0, 0.0]
    robot_linear_vel = 0.0
    robot_angular_vel = 0.0

    robot_path = [[0.0,0.0], [(2**2)/4, (2**2)/4], [1 * (2**2), -(2**2)/2], [1 * (2**2) + 0.5, -(2**2)/2], [1*(2**2) + 0.5, -(2**2)/2 + 0.5]]

    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot_teleop')
    InitListeners()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
