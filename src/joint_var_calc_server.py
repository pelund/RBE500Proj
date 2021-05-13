#!/usr/bin/env python
##takes in end effector locations and returns joint values
import rospy
from rbe_proj.srv import JointVarCalc, JointVarCalcResponse
from math import atan2
from math import sqrt

def handle_joint_var_calc(req):
    xc = req.xc
    yc = req.yc
    zc = req.zc
    a1 = req.a1
    a2 = req.a2
    d3 = req.d3

    r = (pow(xc, 2) + pow(yc, 2) - pow(a1, 2) - pow(a2, 2)) / 2*a1
    # print(r)
    D1_b = -(2 * r * xc)
    D1_a = pow(xc, 2)+pow(yc, 2)
    D1_c = pow(r, 2)-pow(yc, 2)
    print(D1_a, D1_b, D1_c)
    D1_numerator = -D1_b + sqrt(pow(D1_b, 2) - (4* D1_a * D1_c))
    D1_denominator = 2 * D1_a
    D1 = D1_numerator / D1_denominator
    D12 = (xc - (a1*D1)) / a2
    theta1 = atan2(sqrt(1 - pow(D1, 2)), D1)
    theta12 = atan2(sqrt(1 - pow(D12, 2)), D12)
    theta2 = theta12 - theta1
    d3out = -zc
    print("Returning :", req.xc, req.yc, req.zc, req.a1, req.a2, req.d3, theta1, theta2, d3out)
    return JointVarCalcResponse(theta1,theta2,d3out)

def joint_var_calc_server():
    rospy.init_node('joint_var_calc_server')
    s = rospy.Service('joint_var_calc', JointVarCalc, handle_joint_var_calc)
    print("Ready to do joint variable calculations.")
    rospy.spin()

if __name__ == "__main__":
    joint_var_calc_server()