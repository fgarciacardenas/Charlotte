#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import numpy as np

# Commonly used functions
cos = np.cos
sin = np.sin
pi = np.pi
atan2 = np.arctan2
dot = np.dot
array = np.array
ar = np.arange

def initialize():
    # q: Initial configuration vector [posb,leg1,leg2,leg3,leg4,rotb]
    q = np.zeros(18)
    q[0:3] = [0, 0, 0]
    q[3:6] = np.radians([90, 90, 90])
    q[6:9] = np.radians([90, 90, 90])
    q[9:12] = np.radians([90, 90, 90])
    q[12:15] = np.radians([90, 90, 90])
    q[15:18] = np.radians([0, 0, 0])
    # move_servo(q, 1, 1)
    return q

if __name__ == "__main__":

    rospy.init_node("sendJointsGzNode")
    topic1 = '/robot/joint1_position_controller/command'
    topic2 = '/robot/joint2_position_controller/command'
    topic3 = '/robot/joint3_position_controller/command'
    topic4 = '/robot/joint4_position_controller/command'
    topic5 = '/robot/joint5_position_controller/command'
    topic6 = '/robot/joint6_position_controller/command'
    topic7 = '/robot/joint7_position_controller/command'
    topic8 = '/robot/joint8_position_controller/command'
    topic9 = '/robot/joint9_position_controller/command'
    topic10 = '/robot/joint10_position_controller/command'
    topic11 = '/robot/joint11_position_controller/command'
    topic12 = '/robot/joint12_position_controller/command'
    pub1 = rospy.Publisher(topic1, Float64, queue_size=10, latch=True)
    pub2 = rospy.Publisher(topic2, Float64, queue_size=10, latch=True)
    pub3 = rospy.Publisher(topic3, Float64, queue_size=10, latch=True)
    pub4 = rospy.Publisher(topic4, Float64, queue_size=10, latch=True)
    pub5 = rospy.Publisher(topic5, Float64, queue_size=10, latch=True)
    pub6 = rospy.Publisher(topic6, Float64, queue_size=10, latch=True)
    pub7 = rospy.Publisher(topic7, Float64, queue_size=10, latch=True)
    pub8 = rospy.Publisher(topic8, Float64, queue_size=10, latch=True)
    pub9 = rospy.Publisher(topic9, Float64, queue_size=10, latch=True)
    pub10 = rospy.Publisher(topic10, Float64, queue_size=10, latch=True)
    pub11 = rospy.Publisher(topic11, Float64, queue_size=10, latch=True)
    pub12 = rospy.Publisher(topic12, Float64, queue_size=10, latch=True)
    
    q = initialize()

    j1 = Float64()
    j2 = Float64()
    j3 = Float64()
    j4 = Float64()
    j5 = Float64()
    j6 = Float64()
    j7 = Float64()
    j8 = Float64()
    j9 = Float64()
    j10 = Float64()
    j11 = Float64()
    j12 = Float64()

    j1.data = q[3]
    j2.data = q[4]
    j3.data = q[5]
    j4.data = q[6]
    j5.data = q[7]
    j6.data = q[8]
    j7.data = q[9]
    j8.data = q[10]
    j9.data = q[11]
    j10.data = q[12]
    j11.data = q[13]
    j12.data = q[14]

    pub1.publish(j1)
    pub2.publish(j2)
    pub3.publish(j3)
    pub4.publish(j4)
    pub5.publish(j5)
    pub6.publish(j6)
    pub7.publish(j7)
    pub8.publish(j8)
    pub9.publish(j9)
    pub10.publish(j10)
    pub11.publish(j11)
    pub12.publish(j12)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()