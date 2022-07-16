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
import time
import math

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist

import sys, select, termios, tty

pub=0

robot_position = [0.0, 0.0]
robot_orientation = [0.0, 0.0, 0.0]
current_linear_speed = 0.0
current_angular_speed = 0.0

robot_path = [[0.0,0.0], [(2**2)/4, (2**2)/4], [1 * (2**2), -(2**2)/2], [1 * (2**2) + 0.5, -(2**2)/2], [1*(2**2) + 0.5, -(2**2)/2 + 0.5]]


def InitListeners():	
    global pub
    # rospy.Subscriber("occupancy", OccupancyGrid, update_map)
    rospy.Subscriber('odom', Odometry, UpdateOdom) # robot sensor
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)
	# rospy.Subscriber('/t265/odom/sample', Odometry, update_pos) #https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
    rospy.spin()

def UpdateOdom(odometry):
    UpdatePose(odometry.pose)
    UpdateTwist(odometry.twist)

    global robot_position
    global robot_path
    global pub

    control_speed = current_linear_speed
    control_turn = current_angular_speed
    
    target_pos = robot_path[0]
    robot_heading = robot_orientation[2]

    timestamp = time.time()
    # print(str(target_pos) + " " + str(robot_position) + " " + str(target_heading) + " " + str(robot_heading))

    if((abs(robot_position[0] - target_pos[0]) < 0.05) and (abs(robot_position[1] - target_pos[1]) < 0.05)):
        if(len(robot_path) == 1):
            print("done")
        else:
            robot_path = robot_path[1:]

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

    twist = Twist()
    twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
    print(twist)
    pub.publish(twist)

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
    global current_linear_speed
    global current_angular_speed

    current_linear_speed = twist.twist.linear.x
    current_angular_speed = twist.twist.angular.z

def QuaternionToEuler(quaternion):
    roll_x = math.atan2(2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z), 1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y))
    
    val = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
    val = 1.0 if val > 1.0 else val
    val = -1.0 if val < -1.0 else val
    pitch_y = math.asin(val)

    yaw_z = math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x *quaternion.y), 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))

    return [roll_x, pitch_y, yaw_z]

if __name__=="__main__":
    global robot_position
    global robot_orientation
    global current_linear_speed
    global current_angular_speed
    global robot_path 

    robot_position = [0.0, 0.0]
    robot_orientation = [0.0, 0.0, 0.0]
    current_linear_speed = 0.0
    current_angular_speed = 0.0

    robot_path = [[0.0,0.0], [(2**2)/4, (2**2)/4], [1 * (2**2), -(2**2)/2], [1 * (2**2) + 0.5, -(2**2)/2], [1*(2**2) + 0.5, -(2**2)/2 + 0.5]]

    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot_teleop')
    InitListeners()
    timestamp = time.time()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

