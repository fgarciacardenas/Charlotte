#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from markers import *
from lab2functions import *


class Joystick(object):
    def __init__(self):
        self.sub = rospy.Subscriber('joy', Joy, self.callback)
        self.axes = 6*[0.,]
        self.buttons = 6*[0.,]

    def callback(self, msg):
        self.axes = msg.axes
        self.buttons = msg.buttons

rospy.init_node("testInvKine")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)

joystick = Joystick()

bmarker      = BallMarker(color['RED'])
bmarker_des  = BallMarker(color['GREEN'])

# Joint names
jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Desired position
xd = np.array([0.5,0.5,0.5])
# Initial configuration
q0 = np.array([0.0, -1.0, 1.7, -2.2, -1.6, 0.0])
# Inverse kinematics
q = ikine_ur5(xd, q0)

# Resulting position (end effector with respect to the base link)
T = fkine_ur5(q)
print('Obtained value:\n', np.round(T,3))

# Red marker shows the achieved position
bmarker.xyz(T[0:3,3])
# Green marker shows the desired position
bmarker_des.xyz(xd)

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Asignar valores al mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q

# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():
    speed = 0.0001 + (0.001 - 0.0001) * ((joystick.axes[3] - (-1)) / (1 - (-1)))

    # Desired position
    xdnew = np.array([xd[0] + joystick.axes[0]*speed, xd[1] + joystick.axes[1]*speed, xd[2] + joystick.axes[5]*speed])
    qnew = ikine_ur5(xdnew, q)

    if qnew is not False:
        xd = xdnew
        q = qnew

    T = fkine_ur5(q)
    
    # Red marker shows the achieved position
    bmarker.xyz(T[0:3,3])
    # Green marker shows the desired position
    bmarker_des.xyz(xd)
    
    # Resulting position (end effector with respect to the base link)
    jstate.position = q
    
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    bmarker_des.publish()

    # Wait for the next iteration
    rate.sleep()
