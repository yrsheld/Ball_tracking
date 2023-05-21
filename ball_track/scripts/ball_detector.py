#!/usr/bin/env python
import numpy as np
import rospy
import cv2
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ball_detector:
    def __init__(self, imgTopic):
        rospy.init_node('ball_detector', anonymous=True)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(imgTopic, Image, self.img_cb)
        self.pub = rospy.Publisher('/processedImg', Image, queue_size=1)

    def img_cb(self, msg):
        '''
        Detect ball 2D and draw on image
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
            circles = np.uint16(np.around(circles))
            for x, y, r in circles[0,:]:
                cv2.circle(img_bgr, (x, y), r, (0, 255, 0), 2)

        # publish the drawned image
        try:
            self.pub.publish(self.bridge.cv2_to_imgmsg(img_bgr, "bgr8"))
        except CvBridgeError as e:
            print(e)

if __name__=='__main__':
    try:
        imgTopic = rospy.get_param('imgTopic', 'image/raw')
        ball_detector(imgTopic)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
            
    except rospy.ROSInterruptException as e:
        print(e)