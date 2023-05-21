#!/usr/bin/env python
import rospy
import math
import time
from gazebo_msgs.msg import ModelState

def ballMove():
    rospy.init_node('ballMove', anonymous=True)
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    rate = rospy.Rate(10)

    # ball trajectory
    period = 100.0
    omega = 2*math.pi/period
    radius = 3.0

    m = ModelState()
    m.model_name = 'cricket_ball'
        
    startTime = time.time()

    while not rospy.is_shutdown():
        angle = (time.time() - startTime) * omega
        m.pose.position.x = radius * math.sin(angle)
        m.pose.position.y = radius * math.cos(angle)
            
        pub.publish(m)
        rate.sleep()

if __name__ == '__main__':
    try:
        ballMove()
    except rospy.ROSInterruptException as e:
        print(e)