from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray
#from std_msgs.msg import String

from math_functions import *
import std_msgs.msg as stdmsgs
import rospy
import math

state = ""
destination = Point(x=0, y=0, z=0)
sleep_amount = 0.5
robot_position = Point(x=0, y=0, z=0)
print("init")

def InitNode():
    global destination
    global state
    global dest_publisher
    global mode_publisher
    global sleep_amount

    rospy.init_node('turtlebot_controller')
    dest_publisher = rospy.Publisher('turtlebot_controller/destination', Point, queue_size=5)
    mode_publisher = rospy.Publisher('turtlebot_controller/mode', stdmsgs.String, queue_size = 5)
    
    rospy.Subscriber('/t265/odom/sample', Odometry, UpdateOdom)
    rospy.Subscriber('object_depth/detection', Point, UpdateState)

    # target_x = input("Target x: ")
    # target_y = input("Target y: ")

    rospy.sleep(sleep_amount)
    rospy.spin()

def UpdateOdom(odometry):
    '''
    Updates global robot position variable
    '''
    global robot_position
    global robot_orientation
    global destination
    global state
    global sleep_amount

    robot_position = odometry.pose.pose.position
    orientation_quaternion = odometry.pose.pose.orientation
    orientation_euler = QuaternionToEuler(orientation_quaternion)
    # print(orientation_euler)
    robot_orientation = orientation_euler

    arrival_dist = 0.1

    if state == "drive_to_suitcase" and abs(robot_position.x - destination.x) < arrival_dist and abs(robot_position.y - destination.y) < arrival_dist:
        state = "follow_person"
        print("follow")
        sleep_amount = 0.05
    elif state == "return" and abs(robot_position.x - destination.x) < arrival_dist and abs(robot_position.y - destination.y) < arrival_dist:
        state = "stop"
        sleep_amount = 10
        print("stop")
    else:
        sleep_amount = 0.5

    if state == "":
        destination = Point(x = float(0), y = float(0), z = 0)
        mode_publisher.publish(stdmsgs.String(data="finder"))
    elif state == "drive_to_suitcase":
        mode_publisher.publish(stdmsgs.String(data="drive"))
    elif state == "follow_person":
        mode_publisher.publish(stdmsgs.String(data="follower"))
    elif state == "return":
        mode_publisher.publish(stdmsgs.String(data="drive"))
    elif state == "stop":
        mode_publisher.publish(stdmsgs.String(data = "stop"))
    dest_publisher.publish(destination)

    # orientation_quaternion = odometry.pose.pose.orientation
    # orientation_euler = QuaternionToEuler(orientation_quaternion)
    # # print(orientation_euler)
    # robot_orientation = orientation_euler

def UpdateState(point):
    global destination
    global state
    global person_found
    global mode_publisher
    global dest_publisher

    if state == "":
        if int(point.z) == 11:
            suitcase_pos = point

            follow_dist = 0.2

            x_dist = suitcase_pos.x - robot_position.x
            y_dist = suitcase_pos.y - robot_position.y
            vector_heading = math.atan2(y_dist, x_dist)
            true_heading = robot_orientation.z + vector_heading

            new_x_dist = x_dist - follow_dist * math.cos(true_heading)
            new_y_dist = y_dist - follow_dist * math.sin(true_heading)

            new_dest = Point(x = robot_position.x + new_x_dist, y = robot_position.y + new_y_dist, z = suitcase_pos.z)

            destination = new_dest

            state = "drive_to_suitcase"
            print("to suitcase")

    elif state == "follow_person":
        if int(point.z) == 1:
            suitcase_pos = point

            follow_dist = 0.2

            x_dist = suitcase_pos.x - robot_position.x
            y_dist = suitcase_pos.y - robot_position.y
            vector_heading = math.atan2(y_dist, x_dist)
            true_heading = robot_orientation.z + vector_heading

            new_x_dist = x_dist - follow_dist * math.cos(true_heading)
            new_y_dist = y_dist - follow_dist * math.sin(true_heading)

            new_dest = Point(x = robot_position.x + new_x_dist, y = robot_position.y + new_y_dist, z = suitcase_pos.z)

            print("follow")
            destination = new_dest
            
        elif int(point.z) == 15:
            state = "return"
            print("return")
            destination = Point(x = 0, y = 0, z = 0)
        

if __name__=="__main__":
    InitNode()
