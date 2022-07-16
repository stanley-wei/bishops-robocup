from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import rospy

def InitNode():
    rospy.init_node('turtlebot_controller')
    dest_publisher = rospy.Publisher('turtlebot_controller/destination', Point, queue_size=5)
    
    target_x = input("Target x: ")
    target_y = input("Target y: ")

    while True:
        dest_point = Point(x = float(target_x), y = float(target_y), z = 0)
        dest_publisher.publish(dest_point)

        rospy.sleep(2.0)

if __name__=="__main__":
    InitNode()