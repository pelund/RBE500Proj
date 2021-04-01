#! /usr/bin/env python

#!/usr/bin/env python

from __future__ import print_function

from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def calculate_joint_angles(req):
    xc = 3
yc = 1
zc = 5

a1 = 2
a2 = 2
d3 = 8

r = (pow(xc, 2) + pow(yc, 2) - pow(a1, 2) - pow(a2, 2)) / 2*a1
# print(r)

D1_b = -(2 * r * xc)
D1_a = pow(xc, 2)+pow(yc, 2)
D1_c = pow(r, 2)-pow(yc, 2)
# print(D1_a, D1_b, D1_c)
D1_numerator = -D1_b + sqrt(pow(D1_b, 2) - (4* D1_a * D1_c))
D1_denominator = 2 * D1_a
D1 = D1_numerator / D1_denominator
D12 = (xc - (a1*D1)) / a2                                                                                                        

theta1 = atan2(sqrt(1 - pow(D1, 2)), D1)
theta12 = atan2(sqrt(1 - pow(D12, 2)), D12)  
theta2 = theta12 - theta1
d3 = -zc

print('Theta1, theta2 and d3 are:', theta1, theta2, d3)



import numpy as np
from math import pow, sqrt, atan2



# import rospy
# from std_msgs.msg import String

# def callback(data):
#     rospy.loginfo("I heard %s",data.data)
    
# def listener():
#     rospy.init_node('node_name')
#     rospy.Subscriber("chatter", String, callback)
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()
