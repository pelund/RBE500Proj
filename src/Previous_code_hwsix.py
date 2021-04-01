#! /usr/bin/env python
from hw_six.msg import FKangles 
import numpy as np
import math
from math import pi
import rospy
from geometry_msgs.msg import Pose
from math import sin
from math import cos
from geometry_msgs.msg import Twist

def DH_matrix(theta, d, a, alpha):
    T_theta = [[np.cos(theta), -np.sin(theta), 0, 0],
               [np.sin(theta), np.cos(theta), 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]]
    T_d = [[1, 0, 0, 0],
           [0, 1, 0, 0],
           [0, 0, 1, d],
           [0, 0, 0, 1]]

    T_a = [[1, 0, 0, a],
           [0, 1, 0, 0],
           [0, 0, 1, 0],
           [0, 0, 0, 1]]

    T_alpha = [[1, 0, 0, 0],
               [0, np.cos(alpha), -np.sin(alpha), 0],
               [0, np.sin(alpha), np.cos(alpha), 0],
               [0, 0, 0, 1]]

    T = np.matmul(np.matmul(T_theta, T_d), np.matmul(T_a, T_alpha))
    return T

def Forward_kinematics(msg):
    T = {}
    thetas = np.empty(7)
    theta_vals = [msg.theta1, msg.theta2, msg.theta3, msg.theta4, msg.theta5, msg.theta6]
    D = [475, 0, 0, 720, 0, 0, 85]
    A = [150, -600, 120, 0, 0, 0, 0]
    ALPHAS = [pi / 2, 0, pi/2, 0, pi / 2, -pi / 2, 0]

    thetas[0] = theta_vals[0]               ##theta1
    thetas[1] = theta_vals[1] - (pi / 2)    #theta2
    thetas[2] = theta_vals[2] + pi          #theta3
    thetas[3] = pi/2                        #No joint here
    thetas[4] = theta_vals[3]               #theta4
    thetas[5] = theta_vals[4]               #theta5
    thetas[6] = theta_vals[5]               #theta6

    for i in range(0, len(D)):
        H = DH_matrix(thetas[i], D[i], A[i], ALPHAS[i])
        T['T' + str(i + 1)] = H

    # print(T, "\n")
    End = np.eye(4)
    for i in range(1, 8):
        End = np.dot(End, T['T' + str(i)])
        # print(i)
        print(End)

    End_EFFECTOR_FROM_FK = End
    print(End_EFFECTOR_FROM_FK)

    End_effector_Actual = rospy.Publisher('End_effector_actual', Pose, queue_size=1)

    EEF = Pose()
    EEF.position.x = End_EFFECTOR_FROM_FK[0, -1]
    EEF.position.y = End_EFFECTOR_FROM_FK[1, -1]
    EEF.position.z = End_EFFECTOR_FROM_FK[2, -1]
    End_effector_Actual.publish(EEF)

    print('Verified positions are: ', np.round(EEF.position.x), np.round(EEF.position.y), np.round(EEF.position.z))

def Inverse_Kinematics(msg):

    xc = msg.position.x
    yc = msg.position.y
    zc = msg.position.z

    r = np.absolute(np.sqrt(np.power(xc,2) + np.power(yc,2))) - 150
    s = zc - 475
    rs = np.absolute(np.sqrt(np.power(r,2) + np.power(s,2)))
    h = np.absolute(np.sqrt(np.power(120,2) + np.power((720+85),2)))
    D_alpha = (np.power(600, 2) + np.power(rs, 2) - np.power(h, 2))/(2*600*rs)
    D_beta = (np.power(h, 2) + np.power(600, 2) - np.power(rs, 2))/(2*h*600) 

    alpha = np.arctan2(np.sqrt(1 - np.power(D_alpha, 2)), D_alpha)
    beta = np.arctan2(np.sqrt(1 - np.power(D_beta, 2)), D_beta) 

    q1 =np.arctan2(yc, xc)
    q2 = pi/2  - np.arctan2(s, -r) + alpha
    q3 = np.arctan2(720+85, 120) - (pi - beta)

    print('Inverse angles are: ', q1, q2, q3)

def Inverse_velocity(msg):

    theta1 = 0.5
    theta2 = -0.5
    theta3 = 0.5
    theta4 = 0
    theta5 = 0
    theta6 = 0

    Jv11 = 365.0*sin(theta2 - 1.0*theta1 + theta3 + 0.165) + 300.0*cos(theta1 - 1.0*theta2) - 407.0*cos(theta1 + theta2 + theta3 - 14.2) + 150.0*cos(theta1 + 15.7) - 300.0*cos(theta1 + theta2) + 42.5*sin(theta2 - 1.0*theta1 + theta3)
    Jv12 = - 365.0*sin(theta2 - 1.0*theta1 + theta3 + 0.165) - 300.0*cos(theta1 - 10*theta2) - 407.0*cos(theta1 + theta2 + theta3 - 14.2) - 300.0*cos(theta1 + theta2) - 42.5*sin(theta2 - 1.0*theta1 + theta3)
    Jv13 = - 365.0*sin(theta2 - 1.0*theta1 + theta3 + 0.165) - 407.0*cos(theta1 + theta2 + theta3 - 14.2) - 42.5*sin(theta2 - 10*theta1 + theta3)
    Jv21 = 300.0*sin(theta1 - 1.0*theta2) + 407.0*cos(theta1 + theta2 + theta3 + 0.148) + 150.0*cos(theta1) - 300.0*sin(theta1 + theta2) + 42.5*cos(theta2 - 1.0*theta1 + theta3) + 365.0*sin(theta1 - 1.0*theta2 - 1.0*theta3 + 1.41)
    Jv22 = 407.0*cos(theta1 + theta2 + theta3 + 0.148) - 300.0*sin(theta1 - 1.0*theta2) - 300.0*sin(theta1 + theta2) - 42.5*cos(theta2 - 1.0*theta1 + theta3) - 365.0*sin(theta1 - 1.0*theta2 - 1.0*theta3 + 1.41)
    Jv23 = 407.0*cos(theta1 + theta2 + theta3 + 0.148) - 42.5*cos(theta2 - 1.0*theta1 + theta3) - 365.0*sin(theta1 - 1.0*theta2 - 1.0*theta3 + 1.41)
    Jv31 = 0
    Jv32 = 814.0*cos(theta2 + theta3 + 0.148) - 600.0*sin(theta2)
    Jv33 = 814.0*cos(theta2 + theta3 + 0.148)


    Jv = [[Jv11, Jv12, Jv13],
          [Jv21, Jv22, Jv23],
          [Jv31, Jv32, Jv33]]

    Jv_inverse = np.linalg.inv(Jv)

    q_dot = np.dot(Jv_inverse, [msg.linear.x, msg.linear.y, msg.linear.z])

    print('The joint velocities for first three joints for configuration [q1 q2 q3] = ', theta1, theta2, theta3, 'is: ')
    print('\n', q_dot)

    
if __name__ == '__main__':

    rospy.init_node('hw_six')    
    # print('Please enter the positions:')
    # x, y, z = input().split(' ')

    IK_sub = rospy.Subscriber("inverse_kinematics", Pose, Inverse_Kinematics, queue_size=1)
    FK_sub = rospy.Subscriber("forward_kinematics", FKangles, Forward_kinematics, queue_size=1)
    Inverse_velocity_sub = rospy.Subscriber("inverse_velocity", Twist, Inverse_velocity, queue_size=1)
    rospy.spin()