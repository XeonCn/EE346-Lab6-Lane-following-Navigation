#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy as np
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

status_list = []
status = -1

def callback(data):
    global status_list, status
    # rospy.loginfo(rospy.get_caller_id(), data)
    # flag = data.status_list[-2]
    status_list = data.status_list
    if status_list:
        status = status_list[0].status
        # print(status)
        # print(status_list.status)


rospy.Subscriber('move_base/status', GoalStatusArray, callback)

goalPoints = [ 
    # from point1 to point2, to point3, to point4 and then back to point1
    # position[XYZ] and pose[quaternion]
    # In our map of lab, X-direction is from bottom to top and Y-direction is from right to left
    
    [(4.15, -0.120, 0.0), (0.0, 0.0, 0.71244, 0.70172)],
    [(4.35979514942, 3.90, 0.0), (0.0, 0.0, 0.025369587372, 0.999678140221)],
    [(0.26703, 4.00, 0.0), (0.0, 0.0, -0.716918078661, 0.697157420164)],
	[(0.012, -0.020, 0.0), (0.0, 0.0, 0.73358, -0.67959)],
	[(0.01127, -0.020, 0.0), (0.0, 0.0, 0.73358, 0.67959)]
	
]

def init_send(pose):
	init_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, latch=True, queue_size=1)
	init_msg = PoseWithCovarianceStamped()
	init_msg.header.frame_id = 'map'
	init_msg.pose.pose.position.x = pose[0][0]
	init_msg.pose.pose.position.y = pose[0][1]
	init_msg.pose.pose.position.z = pose[0][2]
	init_msg.pose.pose.orientation.x = pose[1][0]
	init_msg.pose.pose.orientation.y = pose[1][1]
	init_msg.pose.pose.orientation.z = pose[1][2]
	init_msg.pose.pose.orientation.w = pose[1][3]
	init_pub.publish(init_msg)


def goal_send(pose):
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.pose.position.x = pose[0][0]
	goal.target_pose.pose.position.y = pose[0][1]
	goal.target_pose.pose.position.z = pose[0][2]
	goal.target_pose.pose.orientation.x = pose[1][0]
	goal.target_pose.pose.orientation.y = pose[1][1]
	goal.target_pose.pose.orientation.z = pose[1][2]
	goal.target_pose.pose.orientation.w = pose[1][3]
	return goal

def navigation(num):
	client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
	client.wait_for_server(rospy.Duration(3))

	goal = goal_send(goalPoints[num])
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration(120))

	
		

		


if __name__=="__main__":
	rospy.init_node('navigation')
	#init_send(goalPoints[4])	
	#print(status)
    	#rospy.spin()
	#navigation(1)
	for i in range(4):
		rospy.sleep(1)
		navigation(i)
	#rospy.sleep(2)
	#print('start')
	#cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, latch=True, queue_size=1)
	#twist = Twist()
        #twist.linear.x = 1.0
        #twist.angular.z = 0.0
        #cmd_vel_pub.publish(twist)
        #rospy.sleep(10)
				

