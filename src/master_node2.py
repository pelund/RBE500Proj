#!/usr/bin/env python
from os import link
import rospy
from vel_controller import PD_Controller_vel
from joint_controller_all import PD_Controller
from rbe_proj.srv import JointVelRev, JointVelRevResponse, JointVarCalc, JointVarCalcRequest,JointVarCalcResponse
from rbe_proj.srv import JointVelForward, JointVelForwardResponse
from gazebo_msgs.srv import GetJointProperties
from gazebo_msgs.srv import GetLinkState
import os
import numpy as np

#TODO: Move the end effector in positive-y direction

#Call joint_vel_rev_server service to get the reference joint velocities for that end effector velocity

#TODO: make velocity controller able to intake the deisred positions
#TODO: Provide those reference joint velocities to the vel_controller so that it provides the required effort
#TODO: To calculate joint velocities, we need jacobian at that position, so we need to be continously recieving the joint positions

if __name__ == '__main__':
    catkin_pkg = os.getenv('ROS_PACKAGE_PATH').split(':')
    catkin_pkg = str(catkin_pkg[0])
    desired_file = open(catkin_pkg + "/rbe_proj/src/velocity_desired_y.txt", "r+")
    current_file = open(catkin_pkg + "/rbe_proj/src/velocity_current_y.txt", "r+")
    current_file = open(catkin_pkg + "/rbe_proj/src/velocity_current_x.txt", "r+")
    current_file = open(catkin_pkg + "/rbe_proj/src/velocity_desired_x.txt", "r+")
    desired_file.truncate(0)
    desired_file.close()
    current_file.truncate(0)
    current_file.close()

    rospy.init_node('master_node')
    end_vel = rospy.ServiceProxy('joint_vel_forward', JointVelForward)
    joint_vel_rev = rospy.ServiceProxy('joint_vel_rev', JointVelRev)           #Gets joint velocities
    joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties) #Gets joint positions 
    link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)    #Gets end effector positions and velocities (or any other link, depending on how you use it)

    
    #Trajectory generation approach =========================================================================#

    end_eff_actual = link_state("link_6", "link")                         #Gets the end_effector position
    y_actual = end_eff_actual.link_state.pose.position.y
    x_actual = end_eff_actual.link_state.pose.position.x
    z_actual = end_eff_actual.link_state.pose.position.z

    yi = y_actual         #Initial velocity
    yf = y_actual + 1    
    tf = 5

    pd1 = PD_Controller('joint_2', 10, 120)
    pd2 = PD_Controller('joint_5', 10, 40)
    pd3 = PD_Controller('joint_6', 15, 20)
    i = yi

    while (i > yi-4):
        #create controller
        end_eff_actual = link_state("link_6", "link")                         #Gets the end_effector position
        y_actual2 = end_eff_actual.link_state.pose.position.y
        x_actual2 = end_eff_actual.link_state.pose.position.x
        z_actual2 = end_eff_actual.link_state.pose.position.z

        joint_var_calc = rospy.ServiceProxy('joint_var_calc', JointVarCalc)
        IK = JointVarCalcResponse()
        IK = joint_var_calc(x_actual,i,z_actual, 5.5, 6, 0)
        t1 = IK.theta1
        t2 = IK.theta2
        d3 = IK.d3out

        current_joint_2properties = joint_properties('joint_2')                 #Stored the current joint poistions and velocities
        current_theta1 = current_joint_2properties.position[0]
        current_vtheta1 = current_joint_2properties.rate[0]

        current_joint_5properties = joint_properties('joint_5')
        current_theta2 = current_joint_5properties.position[0]
        current_vtheta2 = current_joint_5properties.rate[0]

        current_joint_6properties = joint_properties('joint_6')               
        current_d3 = current_joint_6properties.position[0]
        current_vd3 = current_joint_6properties.rate[0]

        pd1.PD(current_vtheta1, t1)                               #Controllers Drive the joints to desired velocity
        pd2.PD(current_vtheta2, t2)
        rospy.sleep(0.1)
        i -= 0.1
        

        #wait a second
        #destroy the PD controller




    # while(vy != 1):
    #     current_joint_2properties = joint_properties('joint_2')                 #Stored the current joint poistions and velocities
    #     current_theta1 = current_joint_2properties.position[0]
    #     current_vtheta1 = current_joint_2properties.rate[0]

    #     current_joint_5properties = joint_properties('joint_5')
    #     current_theta2 = current_joint_5properties.position[0]
    #     current_vtheta2 = current_joint_5properties.rate[0]

    #     current_joint_6properties = joint_properties('joint_6')               
    #     current_d3 = current_joint_6properties.position[0]
    #     current_vd3 = current_joint_6properties.rate[0]

    #     print('Current velocities for q1, q2 and their current postiions are: ', current_vtheta1, current_vtheta2, current_theta1, current_theta2)

    #     desired_jvs = JointVelRevResponse()
    #     desired_jvs = joint_vel_rev(0, 1, 0, current_theta1, current_theta2)    #Stores the desired joint velocities
    #     desired_vtheta1 = desired_jvs.vtheta1
    #     desired_vtheta2 = desired_jvs.vtheta2
    #     desired_vd3 = desired_jvs.vd3   

    #     print('Desired velocities for q1, q2: ', desired_vtheta1, desired_vtheta2)

    #     PDC1 = PD_Controller_vel('joint_2', 20, 8)                            #the controlelrs needed to control the joint velocities: default values: 1: (20, 8), 2: (10, 4), 3: (1, 1)
    #     PDC2 = PD_Controller_vel('joint_5', 10, 4)
    #     PDC3 = PD_Controller_vel('joint_6', 1, 1)

    #     PDC1.PD(current_vtheta1, desired_vtheta1)                               #Controllers Drive the joints to desired velocity
    #     PDC2.PD(current_vtheta2, desired_vtheta2)

    #     # desired_file = open(catkin_pkg + "/rbe_proj/src/velocity_desired_y.txt", "a")
    #     # desired_file.write(str(1) + '\n')
    #     # desired_file.close()

    #     # current_file =  desired_file = open(catkin_pkg + "/rbe_proj/src/velocity_current_y.txt", "a")
    #     # current_file.write(str(vx_actual) + '\n')
    #     # current_file.close()

    #     # desired_file = open(catkin_pkg + "/rbe_proj/src/velocity_desired_x.txt", "a")
    #     # desired_file.write(str(0) + '\n')
    #     # desired_file.close()

    #     # current_file =  desired_file = open(catkin_pkg + "/rbe_proj/src/velocity_current_x.txt", "a")
    #     # current_file.write(str(vx_actual) + '\n')
    #     # current_file.close()


    #     current_end_vels = JointVelForwardResponse()
    #     current_end_vels = end_vel(current_vtheta1, current_vtheta2, current_vd3, current_theta1, current_theta2)             #Get the current  end effector velocities 
    #     vx = current_end_vels.vx
    #     # vy = current_end_vels.vy
    #     vy = vy_actual
    #     vz = current_end_vels.vz

    #     print("Velocity error of vy and vx, vz are: ", (vy_actual-1), vx_actual, vz_actual )
    #     rospy.sleep(0.1)
    #     count += 1
    






        




    
    