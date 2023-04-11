#!/usr/bin/env python3

# GITHUB LINK - https://github.com/aashrita-chemakura/PathPlanning_Project3_Phase2.git
# DONE BY 
# Jayasuriya Suresh
# Aashrita Chemakura

from math import atan2
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import rospy

# defining origin
x, y, yaw = 0.0, 0.0, 0.0

def callback(msg):
    global x, y, yaw
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                          msg.pose.pose.orientation.y,
                                          msg.pose.pose.orientation.z,
                                          msg.pose.pose.orientation.w])

def controller(path):
    rospy.init_node("a_star")
    sub_vel = rospy.Subscriber("/odom", Odometry, callback)
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rate = rospy.Rate(4)
    goal = Point()
    speed = Twist()
    for current_x, current_y in path:
        goal.x, goal.y = current_x, -current_y
        while not rospy.is_shutdown():
            if abs(x - goal.x) < 0.1 and abs(y - goal.y) < 0.1:
                speed.linear.x, speed.angular.z = 0.0, 0.0
                pub_vel.publish(speed)
                rate.sleep()
                break
            else:
                dx, dy = goal.x - x, goal.y - y
                angle_to_achieve = atan2(dy, dx)
                if abs(angle_to_achieve - yaw) > 0.1:
                    speed.linear.x, speed.angular.z = 0.0, 0.1 if angle_to_achieve - yaw > 0 else -0.1
                else:
                    speed.linear.x, speed.angular.z = 0.2, 0.0
                pub_vel.publish(speed)
                rate.sleep()
