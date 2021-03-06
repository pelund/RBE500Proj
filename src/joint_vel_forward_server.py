#!/usr/bin/env python
##takes in end joint info and returns end effector velocities
import rospy
from rbe_proj.srv import JointVelForward, JointVelForwardResponse
from math import atan2
from math import sqrt
import numpy

def handle_joint_vel_forward(req):
    vtheta1 = req.vtheta1
    vtheta2 = req.vtheta2
    vd3 = req.vd3

    theta1 = req.theta1
    theta2 = req.theta2

    a1 = 6
    a2 = 3.75
    L3 = 0    #changed from 0.6 to 3
    J = numpy.array([[((-1*a1*numpy.sin(theta1))-(a2*numpy.sin(theta1+theta2))), -1*a2*numpy.sin(theta1 + theta2), 0],[(a1*numpy.cos(theta1))+(a2*numpy.cos(theta1+theta2)), a2 * numpy.cos(theta1 + theta2), 0] ,[0,0,-1]])
    q = numpy.array([[vtheta1],[vtheta2],[vd3]])
    # p = numpy.multiply(J,q)
    p = numpy.dot(J, q)
    vx = p[0]
    vy = p[1]
    vz = p[2]
    return JointVelForwardResponse(vx,vy,vz)

def joint_vel_forward_server():
    rospy.init_node('joint_vel_forward_server')
    s = rospy.Service('joint_vel_forward', JointVelForward, handle_joint_vel_forward)
    print("Ready to do jvf calculations.")
    rospy.spin()

if __name__ == "__main__":
    joint_vel_forward_server()
