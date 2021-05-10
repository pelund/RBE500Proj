from __future__ import print_function

import sys
import rospy
from rbe_proj.srv import *

#Takes in joint velocities and returns end effector velocities
def joint_var_calc_client(vtheta1, vtheta2,vd3, theta1, theta2):
    rospy.wait_for_service('joint_vel_forward')
    try:
        joint_vel_forward = rospy.ServiceProxy('joint_vel_forward', JointVelForward)
        
        resp1 = joint_vel_forward(vtheta1, vtheta2,vd3, theta1, theta2) 
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

        

def usage():
    return "%s [vtheta1, vtheta2,vd3]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        vtheta1 = float(sys.argv[1])
        vtheta2 = float(sys.argv[2])
        vd3 = float(sys.argv[3])
        theta1 = float(sys.argv[4])
        theta2 = float(sys.argv[5])

    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s,%s,%s,"%(vtheta1, vtheta2,vd3,theta1,theta2))
    # print("%s,%s,%s,%s,%s,%s => %s, %s, %s"%(xc, yc, zc,a1,a2,d3, joint_var_calc_client(xc, yc, zc,a1,a2,d3)))
    q1, q2, q3 = joint_vel_forward_client(vtheta1,vtheta2,vd3)
    print(q1, q2, q3)
