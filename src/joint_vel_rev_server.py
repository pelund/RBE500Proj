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
    L3 = .6
    J = numpy.array([[(-1*a1*numpy.sin(theta1)-a2*numpy.sin(theta1+theta2)), -1*a2*numpy.sin(theta1 + theta2), 1],[a1*numpy.cos(theta1)+a2*cnumpy.cos(theta1+theta2), a2 * numpy.sin(theta1 + theta2), 1] ,[1,1,-1]])
    JT = numpy.transpose(J)
    p = numpy.array([[vx],[vy],[vz])
    q = numpy.multiply(JT,p)
    vtheta1 = p[0,0]
    vtheta2 = p[1,0]
    vtheta3 = p[2,0]
    return JointVelRevResponse(vtheta1,vtheta2,vtheta3)

def joint_var_calc_server():
    rospy.init_node('joint_vel_rev_server')
    s = rospy.Service('joint_vel_rev', JointVelRev, handle_joint_vel_rev)
    print("Ready to do joint variable calculations.")
    rospy.spin()

if __name__ == "__main__":
    joint_vel_rev_server()
