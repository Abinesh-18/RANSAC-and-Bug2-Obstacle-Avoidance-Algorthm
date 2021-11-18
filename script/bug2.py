#!/usr/bin/env python
import roslib
roslib.load_manifest('lab2')
import rospy
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import geometry_msgs.msg
#from interactive_markers.interactive_marker_server import *
#from visualization_msgs.msg import *
import tf
import random
import math
import numpy as np
endpoints = []
pos = 0
orien = 0
wall_front = False
wall_left = False
on_the_line = False
def init():
    rospy.init_node('evader', anonymous=True)
    rospy.Subscriber('/base_scan', LaserScan, callback)

def callback(data):
    global ranges, wall_front, wall_left
    range = data.ranges
    range = np.array(range)
    counter = 0
    maxsize = 120
    for i in np.arange(maxsize):
        if range[180-(maxsize/2)+i] < 1:
            counter += 1
    # range
    if counter > 1:
        wall_front = True
    else:
        wall_front = False

    #resetting the counter value for checking wall_left
    counter = 0
    maxsize = 80
    for i in np.arange(maxsize):
        if range[i] < 1:
            counter += 1
    if counter > 10:
        wall_left = True
    else:
        wall_left = False
            

def callbacktruth(data):
    global orien
    global pos
    pos = data.pose.pose.position
    orien = data.pose.pose.orientation

def getvel(follow_type, goal_ang):
 global wall_left, wall_front, on_the_line
 if follow_type == "GoalSeek":
    if on_the_line:
        return min(goal_ang, 1)
    elif wall_front:
        return 1
    else:
        return min(goal_ang, 1)

 elif follow_type == "WallFollow":
    if wall_front:
        return 0.5
    if wall_left:
        return 0
    else:
        return -1 * 0.4

def evader():
    #pub2 = rospy.Publisher('/visualization_marker',Marker, queue_size = 100)
    #marker = Marker()
    #marker.header.frame_id = "/base_link"
    #marker.header.stamp = rospy.Time.now()
    #marker.ns="lines"
 
    #marker.type=marker.LINE_LIST
    #marker.action = marker.ADD
 
    #marker.pose.orientation.x = 0.0
    #marker.pose.orientation.y = 0.0
    #marker.pose.orientation.z = 0.0
    #marker.pose.orientation.w = 1.0
    #marker.scale.x = 0.1
    #marker.scale.y = 0.1
    #marker.scale.z = 1.0
    #marker.color.g = 1.0
    #marker.color.a = 1.0

    global wall_front, orien, pos, on_the_line
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    sub_truth = rospy.Subscriber('/base_pose_ground_truth', Odometry, callbacktruth)
    rate = rospy.Rate(10)
    endpoints = np.array([[-8, -2],[4.5,9.0]])
    #   Line equation from starting point and destination point
    Goal_reached = False
    threshold = 1
    follow_type = "GoalSeek"
    while not Goal_reached:
        # pos, orien
        if orien <> 0:
            global pos
            robot_angle = 2 * math.asin(orien.z)
            dist = math.sqrt((endpoints[1, 0] - pos.x) ** 2 + (endpoints[1, 1] - pos.y) ** 2)
            goal_ang = math.atan((endpoints[1, 1] - pos.y) / (endpoints[1,0]- pos.x)) - robot_angle
            #on_the_line = point_on_the_line(endpoints)
            A = endpoints[0,:]
            B = endpoints[1,:]
            C = np.array([pos.x, pos.y])
            dist_pt = np.cross(B-A, C-A)/np.linalg.norm(B-A)
            aval = abs((A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1])) / 2.0)
            thold = 0.3
            if (aval < thold):
                on_the_line = True
            else:
                on_the_line = False

            twist = Twist()
            print follow_type
            if dist < threshold:
                #The bot must be stopped    
                twist.linear.x = 0 
                twist.angular.z = 0
                break
            else:
                vel = 0.0
                if wall_front:
                    vel = 0.0
                else:
                    vel = 0.5
                twist.linear.x = vel
                if follow_type == "GoalSeek":
                    twist.angular.z =  getvel(follow_type,goal_ang)
                    if wall_front:
                         follow_type = "WallFollow"
                else:
                    twist.angular.z = -1 * getvel(follow_type, goal_ang)
                    if on_the_line and not wall_front:
                        follow_type = "GoalSeek"
            pub_vel.publish(twist)
            rate.sleep()    
          
if __name__ == '__main__':
    try:
        rospy.init_node('evader', anonymous=True)
        # talker()
        init()
        evader()
    except rospy.ROSInterruptException:
        pass

