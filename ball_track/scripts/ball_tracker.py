#!/usr/bin/env python
import numpy as np
import rospy
import time
from geometry_msgs.msg import Point, Twist

class ball_tracker:
    def __init__(self):
        rospy.init_node('ball_tracker', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/ball_pos', Point, self.pos_cb)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        
        self.lastTime = 0.0
        self.timeout = 1.0

        # motion control parameters
        self.linear_speed = 0.6
        self.angular_speed = 0.6
        self.angular_multiplier = 0.5
        self.decay = 0.8

        self.max_zdist = 2.0 # keep at 2.0 meters from ball
        self.target_xdist = 0.0
        self.target_zdist = 0.0


    def timer_cb(self, event):
        '''
        publish velocity command according to target distance
        '''
        cmd_vel = Twist()
        if time.time()- self.lastTime < self.timeout:
            if self.target_zdist > self.max_zdist:
                cmd_vel.linear.x = self.linear_speed
            cmd_vel.angular.z = - self.angular_multiplier * self.target_xdist
        else:
            rospy.loginfo('Searching for target...')
            cmd_vel.angular.z = self.angular_speed
        
        self.pub.publish(cmd_vel)
        
    def pos_cb(self, pos):
        '''
        set target distance according to ball 3D position
        '''
        self.target_xdist = self.target_xdist * self.decay + pos.x * (1-self.decay)
        self.target_zdist = self.target_zdist * self.decay + pos.z * (1-self.decay)

        self.lastTime = time.time()

if __name__=='__main__':
    try:
        ball_tracker()

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
            
    except rospy.ROSInterruptException as e:
        print(e)