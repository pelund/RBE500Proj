#! /usr/bin/env python

#subscribes to the joint values topic ead from gazebo
#calculate end effector pose
#publish pose as a ros ropic (in callback that reads joint values)

import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from math import pi 

def Forward_kinematics(msg):

    theta1 = msg.position[0]
    theta2 = msg.position[1]
    d3 = msg.position[2]

    L1 = 2
    L2 = 2
    L3 = 1

    T = {}
    thetas = np.empty(3)
    theta_vals = [theta1,theta2, theta3]
    D = [L1, 0, d3 + L3]
    A = [L2, L3, 0]
    ALPHAS = [0,pi,0]

    thetas[0] = theta_vals[0]    ##theta1
    thetas[1] = theta_vals[1]    #theta2
    thetas[2] = theta_vals[2]    #theta3


    for i in range(0, len(D)):
        H = DH_matrix(thetas[i], D[i], A[i], ALPHAS[i])
        T['T' + str(i + 1)] = H

    # print(T, "\n")
    End = np.eye(4)
    for i in range(1, 4):
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
if __name__ == '__main__':

    rospy.init_node('fwdkin')
    FK_sub = rospy.Subscriber("joint_states", JointState, Forward_kinematics, queue_size=1)
    rospy.spin()