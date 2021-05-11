#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from rbe_proj.srv import *

#Takes in joint velocities and returns end effector velocities
def joint_vel_rev_client(vtheta1, vtheta2,vd3, theta1, theta2):
    rospy.wait_for_service('joint_vel_rev')
    try:
        joint_vel_forward = rospy.ServiceProxy('joint_vel_rev', JointVelRev)
        
        resp1 = joint_vel_rev(vx, vy,vz, theta1, theta2) 
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

        

def usage():
    return "%s [vtheta1, vtheta2,vd3]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        vx= float(sys.argv[1])
        vy = float(sys.argv[2])
        vz = float(sys.argv[3])
        theta1 = float(sys.argv[4])
        theta2 = float(sys.argv[5])

    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s,%s,%s,"%(vx, vy,vz,theta1,theta2))
    # print("%s,%s,%s,%s,%s,%s => %s, %s, %s"%(xc, yc, zc,a1,a2,d3, joint_var_calc_client(xc, yc, zc,a1,a2,d3)))
    q1, q2, q3 = joint_vel_rev_client(vx,vy,vz,theta1,theta2)
    print(q1, q2, q3)
