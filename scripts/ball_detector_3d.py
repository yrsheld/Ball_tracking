#!/usr/bin/env python
import numpy as np
import math
import rospy
import cv2
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class ball_detector:
    def __init__(self, ballRadius, imgTopic):
        rospy.init_node('ball_detector', anonymous=True)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(imgTopic, Image, self.img_cb)
        self.pub = rospy.Publisher('/ball_pos', Point, queue_size=1)
        self.marker_pub = rospy.Publisher('/ball_marker', Marker, queue_size=1)

        self.ball_radius = ballRadius

        self.camFrame = 'camera_rgb_optical_frame'
        self.camProc = camProc(ballRadius, self.camFrame)
    
    def img_cb(self, msg):
        '''
        Detect ball 2D & 3D position
        '''
        img_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        #blue_lower = np.array([0, 0, 47])
        #blue_higher = np.array([179, 255, 255])
        #blue_mask = cv2.inRange(img_hsv, blue_lower, blue_higher)
        red_lower = np.array([0, 90, 93])
        red_higher = np.array([52, 255, 255])
        red_mask = cv2.inRange(img_hsv, red_lower, red_higher)

        # search for circles
        circles = cv2.HoughCircles(red_mask, cv2.HOUGH_GRADIENT, 1, 100, param1=200, param2=10, minRadius=5, maxRadius=100)
        if circles is not None:
            x, y, r = np.array(circles[0,0]) # take the first circle
            p = self.camProc.convert3D(x, y, r)
            #rospy.loginfo('Ball 2d {}, {}, {}'.format(x, y, r))
            #rospy.loginfo('Ball 3d {}, {}, {}'.format(p.x, p.y, p.z))
            
            self.publishBallPos(p)
        
            
    def publishBallPos(self, p):
        '''
        publish 3d coordinate and marker visualization
        '''
        # publish 3d point
        print('Ball detected at ', p)
        self.pub.publish(p)

        # publish marker
        m = Marker()
        m.header.frame_id = self.camFrame
        m.id = 0
        m.type = Marker.SPHERE
        m.pose.position.x = p.x
        m.pose.position.y = p.y
        m.pose.position.z = p.z
        m.scale.x = m.scale.y = m.scale.z = self.ball_radius * 2
        m.color.r = 0.9
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.action = Marker.ADD

        self.marker_pub.publish(m)

class camProc:
    def __init__(self, ballRadius, cameraFrame):
        # ball size
        self.ballRadius = ballRadius
        
        # camera info
        self.width = 640
        self.height = 480

        self.aspect_ratio = float(self.width) / self.height
        self.hfov = 1.0856  # rad 
        self.vfov = self.hfov / self.aspect_ratio
        
        self.frame = cameraFrame

    def convert3D(self, x_2d, y_2d, r_2d):
        '''
        convert 2d coords to 3d coords
        '''
        p = Point()
        theta_ball = r_2d / self.width * self.hfov
        # Calculate 3d distance to ball
        d = self.ballRadius / math.tan(theta_ball)

        theta_y = (y_2d / self.height - 0.5) * self.vfov
        p.y = d * math.sin(theta_y)
        d_ = d * math.cos(theta_y)

        theta_x = (x_2d / self.width - 0.5) * self.hfov
        p.x = d_ * math.sin(theta_x)
        p.z = d_ * math.cos(theta_x)
        
        return p       
    
if __name__=='__main__':
    try:
        ballRadius = rospy.get_param('ballRadius', 0.015)
        imgTopic = rospy.get_param('imgTopic', 'image/raw')

        ball_detector(ballRadius, imgTopic)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
            
    except rospy.ROSInterruptException as e:
        print(e)