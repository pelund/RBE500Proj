#!/usr/bin/env python
import rospy
from vel_controller import PD_Controller_vel
from rbe_proj.srv import JointVelRev, JointVelRevResponse
from rbe_proj.srv import JointVelForward, JointVelForwardResponse
from gazebo_msgs.srv import GetJointProperties
#TODO: Move the end effector in positive-y direction

#Call joint_vel_rev_server service to get the reference joint velocities for that end effector velocity



#TODO: make velocity controller able to intake the deisred positions
#TODO: Provide those reference joint velocities to the vel_controller so that it provides the required effort
#TODO: To calculate joint velocities, we need jacobian at that position, so we need to be continously recieving the joint positions

if __name__ == '__main__':
    rospy.init_node('master_node')
    end_vel = rospy.ServiceProxy('joint_vel_forward', JointVelForward)
    joint_vel_rev = rospy.ServiceProxy('joint_vel_rev', JointVelRev)          #Gets joitn velocities
    joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties) #Gets joint positions 

    
    # vy_desired = 1
    vy = 0
    count = 0
    while(count < 200):
        current_joint_2properties = joint_properties('joint_2')                 #Stored the current joint poistions and velocities
        current_theta1 = current_joint_2properties.position[0]
        current_vtheta1 = current_joint_2properties.rate[0]

        current_joint_5properties = joint_properties('joint_5')
        current_theta2 = current_joint_5properties.position[0]
        current_vtheta2 = current_joint_5properties.rate[0]

        print('Current velocities for q1, q2: ', current_vtheta1, current_vtheta2)

        current_joint_6properties = joint_properties('joint_6')
        current_d3 = current_joint_6properties.position[0]
        current_vd3 = current_joint_6properties.rate[0]

        desired_jvs = JointVelRevResponse()
        desired_jvs = joint_vel_rev(0, 1, 0, current_theta1, current_theta2)    #Stored the desired joint velocities
        desired_vtheta1 = desired_jvs.vtheta1
        desired_vtheta2 = desired_jvs.vtheta2
        desired_vd3 = desired_jvs.vd3   

        print('Desired velocities for q1, q2: ', desired_vtheta1, desired_vtheta2)

        PDC1 = PD_Controller_vel('joint_2', 30, 100)                            #the controlelrs needed to control the velocities
        PDC2 = PD_Controller_vel('joint_5', 10, 4)
        PDC3 = PD_Controller_vel('joint_6', 30, 100)

        PDC1.PD(current_vtheta1, desired_vtheta1)                               #Drives the joints to desired velocity
        PDC2.PD(current_vtheta2, desired_vtheta2)

        current_end_vels = JointVelForwardResponse()
        current_end_vels = end_vel(current_vtheta1, current_vtheta2, current_vd3, current_theta1, current_theta2)             #Get the current  end effector velocities 
        vx = current_end_vels.vx
        vy = current_end_vels.vy
        vz = current_end_vels.vz
        rospy.sleep(0.1)
        count += 1





        




    
    