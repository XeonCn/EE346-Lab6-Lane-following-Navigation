#!/usr/bin/env python

import cv2
import cv_bridge
import numpy as np
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2.aruco as aruco
DTYPE = np.float32

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
param = aruco.DetectorParameters_create()
class Follower:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('camera/image',
                                          Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                                           Twist, queue_size=10)
        self.starttimer = rospy.get_time()
        self.twist = Twist()
        self.previous_x = 0.0
        self.previous_z = 0.0
        self.start = 0
        self.init = 0
        self.right = 0
        self.left = 0
	self.straight = 0
        self.finish = 0
        self.count = 0
        self.start1 = 0
        self.s0 = rospy.get_time()
        self.s1 = rospy.get_time()
        # self.template = cv2.imread('/home/zeon/Desktop/BEV_1.png', 0).astype(np.uint8)
        self.PID_a = pid_controller(0.003, 0.00, 0)
        self.PID_b = pid_controller(1, 0.00, 0.00)
        self.stopped = False

    def image_callback(self, msg):

        # rate = rospy.Rate(10)

        # np_arr = np.fromstring(msg.data, np.uint8)
        # image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # H2 = np.array([[-0.434,-1.33,229],[0,-2.88,462],[0,-0.00833,1]])
        # homography process
        # BEV = cv2.warpPerspective(image, H2, (320, 240))

        # homography transform process
        top_x = 68  # 102 73     62 10 140 120     50 20 140 120    68 5 140 120
        top_y = 5  # 10 26      40 30 140 120
        bottom_x = 140  # 230
        bottom_y = 120
        # selecting 4 points from the original image
        pts_src = np.array([[160 - top_x, 180 - top_y], [160 + top_x, 180 - top_y], [160 + bottom_x, 120 + bottom_y],
                            [160 - bottom_x, 120 + bottom_y]])
        IMG_H = 240
        IMG_W = 320
        # selecting 4 points from image that will be transformed
        LANE_LEFT = 53.3333
        LANE_RIGHT = 266.6667
        pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])

        # finding homography matrix
        h, status = cv2.findHomography(pts_src, pts_dst)

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 80])
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image_t = cv2.inRange(hsv, lower_black, upper_black)

        # homography process
        BEV = cv2.warpPerspective(image_t, h, (1000, 600))  # 10:6
        # BEV1 = cv2.warpPerspective(image_t, h, (1000, 600)) #10:6

        mask1 = np.copy(BEV)
        mask2 = np.copy(BEV)
        mask3 = np.copy(BEV)
        mask4 = np.copy(BEV)
        h = 600
        w = 1000
        search_top = 3 * h / 10

        # mask1[0:search_top, 0:w] = 0
        # mask1[search_top:h, w/2:w] = 0
        # mask2[0:search_top, 0:w] = 0

        mask1[:, 500:w] = 0
        mask2[:, 0:500] = 0
        mask3[200:h, :] = 0
        mask3[:, 500:w] = 0
        mask4[400:h, :] = 0
        mask4[:, 0:500] = 0
	#M0 = cv2.moments(BEV)
        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)
        M3 = cv2.moments(mask3)
        M4 = cv2.moments(mask4)

        # print np.dtype(np.max(BEV)),np.dtype(np.max(template))
        BEV_b = BEV.copy()
        # print BEV_b.shape,template.shape
        # import pdb;pdb.set_trace()
        # res = cv2.matchTemplate(BEV_b,self.template,cv2.TM_CCOEFF_NORMED)

        # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        #
        # # print max_val
        # if max_val > 0.85:
        #     t = rospy.get_time()
        #     if t - self.s0 > 1:
        #         self.count += 1
        #         self.s0 = t
        #         print self.count
        #self.start = 1
        self.init = 1
        if self.start == 0:
            rospy.sleep(1)
            self.twist.linear.x = 0.21
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(3.5)
            self.twist.linear.x = 0.0
            self.twist.angular.z = -1.5
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1)
            self.twist.linear.x = 0.21
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1.5)
            self.twist.linear.x = 0.21
            self.twist.angular.z = 1.1
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(2.8)
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.start = 1


        if M3['m00'] == 0 and self.left == 0 and self.finish == 0:
	    self.left = 1
            self.s0 = rospy.get_time()
            print('left')
            #rospy.sleep(0.5)
            self.twist.linear.x = 0.21
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1.2)
            self.twist.linear.x = 0.21
            self.twist.angular.z = 1.2
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(3)
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            #rospy.sleep(0.85)
        t = rospy.get_time()
	#print t - self.s0
	#self.start1 = 1
	#if t - self.s0 > 4.2:
		#if self.start1 == 0:
		#	t = self.s0
		#	self.start1 = 1
		#self.straight = 1
	#else:
		#self.straight = 0
	#print self.straight
        if (M1['m00'] > 0 or M2['m00'] > 0) and t - self.s0 > 4.7:
            self.left = 0
            if M1['m00'] == 0:
                cx1 = 150
                cy1 = 300
                cx2 = int(M2['m10'] / M2['m00'])
                cy2 = int(M2['m01'] / M2['m00'])
                fpt_x = (cx1 + cx2) / 2
                fpt_y = (cy1 + cy2) / 2
            elif M2['m00'] == 0:
                cx1 = int(M1['m10'] / M1['m00'])
                cy1 = int(M1['m01'] / M1['m00'])
                cx2 = 850
                cy2 = 300
                fpt_x = (cx1 + cx2) / 2
                fpt_y = (cy1 + cy2) / 2
            else:
                cx1 = int(M1['m10'] / M1['m00'])
                cy1 = int(M1['m01'] / M1['m00'])
                cx2 = int(M2['m10'] / M2['m00'])
                cy2 = int(M2['m01'] / M2['m00'])
                fpt_x = (cx1 + cx2) / 2
                fpt_y = (cy1 + cy2) / 2
            self.twist.linear.x = 0.21
            # print fpt_x,fpt_y
            cv2.circle(BEV, (cx1, cy1), 10, (100, 255, 255), -1)
            cv2.circle(BEV, (cx2, cy2), 10, (100, 255, 255), -1)
            cv2.circle(BEV, (fpt_x, fpt_y), 10, (255, 100, 100), -1)

            err = 10 + w / 2 - fpt_x

            # print err_a
            alpha = -np.arctan2(fpt_x - w / 2, h - fpt_y)
            # beta = math.pi/2 - alpha - math.atan2(m_yc-m_ya,m_xa-m_xc)
            # !
            # print alpha,beta,math.atan2(m_yb-m_ya,m_xa-m_xb)

            self.twist.angular.z = self.PID_b.set_current_error(alpha)
            #print t - self.s0 t - self.s0 > 2  and
            if self.stopped == True:
                self.twist.linear.x = 0.21
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1.15)
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(20)
            self.cmd_vel_pub.publish(self.twist)
            # self.twist.linear.x, self.twist.angular.z
        # else:
        # self.twist.linear.x = self.previous_x
        # self.twist.angular.z = self.previous_z


        self.previous_x = self.twist.linear.x
        self.previous_z = self.twist.angular.z
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, markerID, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=param)
        matrix = np.array([[260.17262,   0.     , 160.76833],
                              [0.     , 260.26526, 119.20188],
                              [0., 0., 1.]])
        dist = np.array([[0.163280, -0.292913, 0.002210, 0.000031, 0.000000]])

        if len(corners) > 0 and not self.stopped:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, matrix, dist)
            (rvec - tvec).any()
            for i in range(rvec.shape[0]):
                aruco.drawDetectedMarkers(image, corners, markerID)
                # corners=markerCorner.reshape((4,2))
                # (topLeft,topRight,bottomRight,bottomleft)=corners
                # topRight = (int(topRight[0]), int(topRight[1]))
                # bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                # bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                # topLeft = (int(topLeft[0]), int(topLeft[1]))
                # cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                # cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                # cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                # cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            distance = int(tvec[0][0][2] * 1000)
            print("[INFO] ArUco marker ID: {}".format(markerID))
            print("distance: ", distance, "mm")
            if distance <= 300:
                self.stopped = True
	thistime = rospy.get_time()
        if thistime - self.starttimer >= 30 and self.right == 0 and M4['m00'] == 0 and self.finish == 0:
            self.right = 1
        if self.right == 1 and M4['m00'] > 0:
            print("detecting right")
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(2)
            self.twist.linear.x = 0.0
            self.twist.angular.z = -1.5
            self.cmd_vel_pub.publish(self.twist)
            self.right = 0
            self.finish = 1
            rospy.sleep(1.1)
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

        #cv2.imshow("w3", mask3)
        cv2.imshow("w4", mask4)
        cv2.imshow("BEV", BEV)
        cv2.waitKey(1)


class pid_controller:

    def __init__(self, p_coef, i_coef, d_coef):
        self.kp = p_coef
        self.ki = i_coef
        self.kd = d_coef
        self._previous_error = 0.0
        self.sum = 0.0

    def set_current_error(self, error):
        output0 = error * self.kp

        error_diff = error - self._previous_error
        output1 = self.kd * error_diff

        self.sum += error
        output2 = self.ki * self.sum

        self._previous_error = error

        output = output0 + output1 + output2

        return output


rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
