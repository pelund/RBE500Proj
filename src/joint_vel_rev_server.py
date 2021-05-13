#!/usr/bin/env python
##takes in end effector velocities and returns joint velocities
import rospy
from rbe_proj.srv import JointVelRev, JointVelRevResponse
from math import atan2
from math import sqrt
import numpy

def handle_joint_vel_rev(req):
    vx = req.vx
    vy = req.vy
    vz = req.vz

    theta1 = req.theta1
    theta2 = req.theta2

    a1 = 5.5
    a2 = 6
    L3 = 3.75    #changed from 0.6 to 3
    J = numpy.array([[((-1*a1*numpy.sin(theta1))-(a2*numpy.sin(theta1+theta2))), -1*a2*numpy.sin(theta1 + theta2), 0],[(a1*numpy.cos(theta1))+(a2*numpy.cos(theta1+theta2)), a2 * numpy.cos(theta1 + theta2), 0] ,[0,0,-1]])
    # JT = numpy.transpose(J)
    p = numpy.array([[vx],[vy],[vz]])
    # q = numpy.multiply(JT,p)
    q = numpy.linalg.solve(J, p)
    print('Jacobian is: ', J)
    vtheta1 = q[0]
    vtheta2 = q[1]
    vtheta3 = q[2]
    return JointVelRevResponse(vtheta1,vtheta2,vtheta3)

def joint_vel_rev_server():
    rospy.init_node('joint_vel_rev_server')
    s = rospy.Service('joint_vel_rev', JointVelRev, handle_joint_vel_rev)
    print("Ready to do joint variable calculations.")
    rospy.spin()

if __name__ == "__main__":
    joint_vel_rev_server()
