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
#from urllib import robotparser
# from numpy import angle
from locale import MON_12
import rospy
import datetime
import math
import numpy as np

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from geometry_msgs.msg import Twist
from math_functions import *

import sys, select, termios, tty

motor_publisher=0

target_point = PoseStamped()
# robot_path = [Point(0.0, 0.0, 0.0)]

integral_heading_error = 0.0

timestamp = datetime.datetime.now()

def InitListeners():	
    global motor_publisher
    global robot_path

    rospy.Subscriber("turtlebot_occupancy/path", Path, UpdatePath)
    # rospy.Subscriber('odom', Odometry, UpdateOdom) # robot sensor
    rospy.Subscriber('/t265/odom/sample', Odometry, UpdateOdom) #https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
   
    motor_publisher = rospy.Publisher('~cmd_vel', Twist, queue_size=5)

    #rospy.sleep(0.25)
    rospy.spin()

def UpdateOdom(odometry):
    UpdatePose(odometry.pose)
    UpdateTwist(odometry.twist)
    Drive()

def Drive():
    global robot_position
    global robot_path
    global motor_publisher
    global integral_heading_error
    global timestamp
    global previous_point

    error_radius = 0.1
    path_updated = False
    while(abs(robot_path[0].pose.position.y - robot_position.y) < error_radius and abs(robot_path[0].pose.position.x - robot_position.x) < error_radius and len(robot_path) > 0):
        previous_point = robot_path[0].pose.position
        robot_path = robot_path[1:]
        print("hi")
        path_updated = True
    
    if path_updated == False:
        previous_point = robot_position

    target_pos = robot_path[0].pose.position

    # if((abs(robot_position.x - target_pos.x) < 0.05) and (abs(robot_position.y - target_pos.y) < 0.05)):
    #     if(len(robot_path) == 1):
    #         print("done")
    #     else:
    #         robot_path = robot_path[1:]
    #         target_pos = robot_path[0].pose.position

    control_speed = robot_linear_vel
    control_turn = robot_angular_vel
    
    robot_heading = robot_orientation.getAngleZ()

    if(len(robot_path) >= 3):
        target_pos = robot_path[2].pose.position
    else:
        target_pos = robot_path[len(robot_path)-1].pose.position

    # print(str(target_pos) + " " + str(robot_position) + " " + str(target_heading) + " " + str(robot_heading))

    if(len(robot_path) > 0):
        # if(len(robot_path) < 3):
        target_heading = (math.atan2((target_pos.y-robot_position.y), (target_pos.x-robot_position.x)))
        v_r_target = 0
        # else:
        #     point_1 = previous_point
        #     #point_1 = robot_position
        #     point_2 = robot_path[0].pose.position
        #     point_3 = robot_path[2].pose.position
        #     point_matrix = np.array([[point_1.x**2 + point_1.y**2, point_1.x, point_1.y, 1], [point_2.x**2 + point_2.y**2, point_2.x, point_2.y, 1], [point_3.x**2 + point_3.y**2, point_3.x, point_3.y, 1]])

        #     if(point_1.x != point_2.x and point_2.x != point_3.x and point_1.y != point_2.y and point_2.y != point_3.y):
        #         # m_11 = np.array([point_matrix[0][1:], point_matrix[1][1:], point_matrix[2][1:]])
        #         # m_12 = np.array([np.concatenate([point_matrix[0][:1], point_matrix[0][2:]]), np.concatenate([point_matrix[1][:1], point_matrix[1][2:]]), np.concatenate([point_matrix[2][:1], point_matrix[2][2:]])])
        #         # m_13 = np.array([np.concatenate([point_matrix[0][:2], point_matrix[0][3:]]), np.concatenate([point_matrix[1][:2], point_matrix[1][3:]]), np.concatenate([point_matrix[2][:2], point_matrix[2][3:]])])
        #         # m_14 = np.array([point_matrix[0][:3], point_matrix[1][:3], point_matrix[2][:3]])

        #         print(point_1)
        #         print(point_2)
        #         print(point_3)
        #         # print(point_matrix)
        #         # print(m_11)
        #         # print(m_12)
        #         # print(m_13)
        #         # print(m_14)
        #         # print(np.linalg.det(m_11))
        #         # print(np.linalg.det(m_12))
        #         # print(np.linalg.det(m_13))
        #         # print(np.linalg.det(m_14))

        #         # x_center = 0.5 * (np.linalg.det(m_12) / np.linalg.det(m_11))
        #         # y_center = -0.5 * (np.linalg.det(m_13) / np.linalg.det(m_11))
        #         # radius = math.sqrt(x_center ** 2 + y_center**2 + (np.linalg.det(m_14) / np.linalg.det(m_11)))

        #         m_1 = (point_2.y - point_1.y) / (point_2.x - point_1.x)
        #         midpoint_x1 = (point_1.x + point_2.x) / 2.0
        #         midpoint_y1 = (point_1.y + point_2.y) / 2.0
        #         b_1 = midpoint_y1 - (-(1.0 / m_1) * midpoint_x1)
        #         m_1_perp = -1.0 / m_1

        #         print(m_1_perp)
        #         print(b_1)


        #         m_2 = (point_3.y - point_2.y) / (point_3.x - point_2.x)
        #         midpoint_x2 = (point_3.x + point_2.x) / 2.0
        #         midpoint_y2 = (point_3.y + point_2.y) / 2.0
        #         b_2 = midpoint_y2 - (-(1.0 / m_2) * midpoint_x2)
        #         m_2_perp = -1.0 / m_2

        #         print(m_2_perp)
        #         print(b_2)

        #         x_center = (b_2 - b_1) / (m_1_perp - m_2_perp)
        #         y_center = m_1_perp * x_center + b_1
        #         radius = math.sqrt((point_1.x - x_center) ** 2 + (point_1.y - y_center) ** 2)

        #         #v_r_target = (2 * robot_linear_vel) / (radius ** 2)
        #         v_r_target = robot_linear_vel / radius

        #         print(x_center)
        #         print(y_center)
        #         print(radius)
        #         target_heading = math.atan2((robot_position.y - y_center), robot_position.x - x_center)
        #         to_next_point = math.atan2((point_2.y - y_center), point_2.x - x_center)

        #         while abs(target_heading) > 2 * math.pi:
        #             target_heading = (2 * math.pi) - target_heading
        #         if abs(target_heading) > math.pi:
        #             target_heading = math.pi - target_heading

        #         while abs(to_next_point) > 2 * math.pi:
        #             to_next_point = (2 * math.pi) - to_next_point
        #         if abs(to_next_point) > math.pi:
        #             to_next_point = math.pi - to_next_point

        #         offset = to_next_point - target_heading

        #         while abs(offset) > 2 * math.pi:
        #             offset = (2 * math.pi) - offset
        #         if abs(to_next_point) > math.pi:
        #             offset = math.pi - offset
                
        #         if offset < 0:
        #             target_heading += -math.pi/2
        #         else:
        #             target_heading += math.pi/2

        #         robot_m = math.tan(robot_heading)
        #         robot_b = robot_position.y- robot_position.x * robot_m
        #         if(x_center * robot_m + robot_b) > y_center:
        #             v_r_target = v_r_target * -1
        #         if robot_heading > math.pi / 2 or robot_heading < -math.pi/2:
        #             v_r_target = v_r_target * -1
        #     else:
        #          target_heading = (math.atan2((target_pos.y-robot_position.y), (target_pos.x-robot_position.x)))
        #          v_r_target = 0

        K_p = 0.5
        K_i = 0
        K_d = 0.0

        while abs(target_heading) > 2 * math.pi:
            target_heading = (2 * math.pi) - target_heading
        if abs(target_heading) > math.pi:
            if target_heading < 0:
                target_heading = target_heading + 2 * math.pi
            else:
                target_heading = target_heading - 2 * math.pi

        angle_error = target_heading - robot_heading
        angle_error_orig = angle_error
        # angle_error = angle_error - ((int(angle_error / (math.pi * 2))) * (math.pi * 2))
        while abs(angle_error) > 2 * math.pi:
            angle_error = (2 * math.pi) - angle_error
        if abs(angle_error) > math.pi:
            if angle_error < 0:
                angle_error = angle_error + 2 * math.pi
            else:
                angle_error = angle_error - 2 * math.pi
        
        # if(abs(angle_error) > math.pi):
        #     if(angle_error)

        integral_heading_error += float((datetime.datetime.now() - timestamp).total_seconds())/60.0 * angle_error

        target_turn = v_r_target
        target_turn = K_p * angle_error
        target_turn += K_i * integral_heading_error
        if(target_turn > 1.2):
            target_turn = 1.2
        elif(target_turn < -1.2):
            target_turn = -1.2

        if(len(robot_path)<=5):
            control_speed = max(robot_linear_vel - 0.05, 0.2)
        elif (abs(target_turn) > 0.7):
            # control_speed = max(robot_linear_vel - 0.1, 0)
            control_speed = min(0.3, 0.3 * (0.6 / abs(target_turn)))
        else:
            control_speed = 0.3

        print("Current position: (" + str(robot_position.x) + ", " + str(robot_position.y) + ")")
        print("Target position: (" + str(target_pos.x) + ", " + str(target_pos.y) + ")")
        print(robot_heading)
        print(target_heading)
        print(angle_error)
        print("Control: " + str(control_speed) + " speed " + str(target_turn) + " turn")
        print("Target: " + str(target_turn) + " turn")
        # if status == "turning" and (control_speed) != 0:
        #     control_turn = 0
        # elif status == "driving" and (control_turn) != 0:
        #     control_speed = 0
    else:
        control_speed = 0
        control_turn = 0

    twist = Twist()
    twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = target_turn
    # print(twist)
    motor_publisher.publish(twist)

def UpdatePose(pose):
    global robot_position
    global robot_orientation

    robot_position = pose.pose.position

    orientation = pose.pose.orientation
    orientation_euler = QuaternionToEuler(orientation)
    robot_orientation = orientation_euler

def UpdateTwist(twist):
    global robot_linear_vel
    global robot_angular_vel

    robot_linear_vel = twist.twist.linear.x
    robot_angular_vel = twist.twist.angular.z

def UpdatePath(path):
    global robot_path

    robot_path = path.poses
    # Drive()

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
    global previous_point

    robot_position = [0.0, 0.0]
    robot_orientation = [0.0, 0.0, 0.0]
    robot_linear_vel = 0.0
    robot_angular_vel = 0.0
    previous_point = Point()
    previous_point.x = 0; previous_point.y = 0; previous_point.z = 0;

    robot_path = [[0.0,0.0], [(2**2)/4, (2**2)/4], [1 * (2**2), -(2**2)/2], [1 * (2**2) + 0.5, -(2**2)/2], [1*(2**2) + 0.5, -(2**2)/2 + 0.5]]

    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot_teleop')
    InitListeners()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
