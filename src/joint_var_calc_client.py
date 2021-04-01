from __future__ import print_function

import sys
import rospy
from rbe_proj.srv import JointVarCalc


def joint_var_calc_client(xc, yc, zc,a1,a2,d3):
    rospy.wait_for_service('joint_var_calc')
    try:
        joint_var_calc = rospy.ServiceProxy('joint_var_calc', JointVarCalc)
        
        resp1 = joint_var_calc(xc, yc, zc,a1,a2, d3)        # changed from d3out to d3
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

        

def usage():
    return "%s [xc, yc, zc, a1, a2, d3]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 7:
        xc = int(sys.argv[1])
        yc = int(sys.argv[2])
        zc = int(sys.argv[3])
        a1 = int(sys.argv[4])
        a2 = int(sys.argv[5])
        d3 = int(sys.argv[6])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s,%s,%s,%s,%s,%s"%(xc, yc, zc,a1,a2,d3))
    print("%%s,%s,%s,%s,%s,%s => %s, %s, %s"%(xc, yc, zc,a1,a2,d3, joint_var_calc_client(xc, yc, zc,a1,a2,d3)))