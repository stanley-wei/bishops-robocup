#!/usr/bin/env python

#from urllib import robotparser

from tokenize import endpats
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

from nav_msgs.msg import OccupancyGrid, Odometry, Path, MapMetaData
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from math_functions import *
from map_functions import *

import sys, select, termios, tty
import rospy
import math
import time
import cv2

path_publisher = 0

occupancy_map = OccupancyGrid()
occupancy_resolution = 1.0
occupancy_metadata = MapMetaData()
occupancy_origin = Pose()

target_point = Point()


def InitListeners():
    '''
    Initializes ROS listeners and subscribers
    /occupancy: realsense occupancy node
    /t265/odom/sample: intel realsense t265 odometry (camera position_
    /turtlebot_controller/destination: node launched by turtlebot_controller.py, gives destination point
    
    /turtlebot_occupancy/patH: node launched by this program (publishes path based on occupancy)
    '''	
    global path_publisher

    # rospy.Subscriber('odom', Odometry, UpdateOdom) # robot sensor
    rospy.Subscriber("occupancy", OccupancyGrid, UpdateMap)
    rospy.Subscriber('/t265/odom/sample', Odometry, UpdateOdom) #https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
    rospy.Subscriber('turtlebot_controller/destination', Point, UpdateDest)

    path_publisher = rospy.Publisher('turtlebot_occupancy/path', Path, queue_size=5)

    rospy.sleep(0.5)
    rospy.spin()

def UpdateOdom(odometry):
    '''
    Updates global robot position variable
    '''
    global robot_position
    global robot_orientation

    robot_position = odometry.pose.pose.position

    # orientation_quaternion = odometry.pose.pose.orientation
    # orientation_euler = QuaternionToEuler(orientation_quaternion)
    # # print(orientation_euler)
    # robot_orientation = orientation_euler

def FindPath(maze, start, end):
    '''
    Outputs path through grid as list of points
    '''

    grid = Grid(matrix = maze)

    finder = AStarFinder(diagonal_movement = DiagonalMovement.always)
    start_point = grid.node(start[1], start[0])
    end_point = grid.node(end[1], end[0])
    print("through")
    path, runs = finder.find_path(start_point, end_point, grid)
    # print(grid.grid_str(path=path, start=start, end = end))

    return path, runs

def UpdateMap(occupancy_grid):
    global occupancy_map
    global occupancy_resolution
    global occupancy_origin
    global occupancy_metadata
    global path_publisher
    global target_point

    # map = np.array(occupancy_grid.data)
    # occupancy_map = ProcessMap(map)
    processed_grid = ProcessMap(occupancy_grid, target_point)
    occupancy_map = processed_grid.data

    occupancy_resolution = processed_grid.info.resolution
    occupancy_metadata = processed_grid.info
    occupancy_origin = occupancy_grid.info.origin.position

    robot_grid_position = PointToGridCoords(robot_position)
    target_grid_position = PointToGridCoords(target_point)

    print("robot " + str(robot_grid_position))
    print(processed_grid.data[robot_grid_position[0], robot_grid_position[1]])
    print("target: " + str(target_grid_position))
    print(processed_grid.data[target_grid_position[0], target_grid_position[1]])
    # path = FindPath(np.ones((processed_grid.info.height, processed_grid.info.height)), robot_grid_position, target_grid_position)
    path = FindPath(np.reshape(occupancy_map, (processed_grid.info.height, processed_grid.info.width)), robot_grid_position, target_grid_position)

    path_message = Path()
    print("out")
    print(len(path[0]))
    # print(path[0])
    for i in range(min(10, len(path[0]))):
        to_add = PoseStamped()
        to_add.header.seq = i

        pose_to_add = Pose()
        pose_to_add.position = GridCoordsToPoint(path[0][i])
        # pose_to_add.orientation = Quaternion(x = 0, y = 0, z = 0, w = 0)
        to_add.pose = pose_to_add

        path_message.poses.append(to_add)

    print(str(i + 1) + " points added")
    path_publisher.publish(path_message)

def PointToGridCoords(point):
    global occupancy_origin
    global occupancy_resolution

    x_diff = point.x - occupancy_origin.x
    x_grid_dist = int(RoundTo(x_diff, occupancy_resolution)/occupancy_resolution)

    y_diff = point.y - occupancy_origin.y
    y_grid_dist = -int(RoundTo(y_diff, occupancy_resolution)/occupancy_resolution)

    grid_position = [x_grid_dist, y_grid_dist]

    return grid_position

def GridCoordsToPoint(point_2d):
    global occupancy_origin
    global occupancy_resolution

    point = Point()
    point.x = occupancy_origin.x + float(point_2d[1]) * occupancy_resolution
    point.y = occupancy_origin.y - float(point_2d[0]) * occupancy_resolution
    point.z = 0

    return point

def UpdateDest(point):
    global target_point

    target_point = point

if __name__=="__main__":
    global robot_position
    global robot_orientation
    global robot_linear_vel
    global robot_angular_vel
    global robot_path 

    robot_position = Point(x = 0.0, y = 0.0, z = 0.0)
    robot_orientation = OrientationEuler(0.0, 0.0, 0.0)

    robot_position = [0.0, 0.0]
    robot_orientation = [0.0, 0.0, 0.0]
    robot_linear_vel = 0.0
    robot_angular_vel = 0.0

    robot_path = [[0.0,0.0], [(2**2)/4, (2**2)/4], [1 * (2**2), -(2**2)/2], [1 * (2**2) + 0.5, -(2**2)/2], [1*(2**2) + 0.5, -(2**2)/2 + 0.5]]

    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot_occupancy')
    InitListeners()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
