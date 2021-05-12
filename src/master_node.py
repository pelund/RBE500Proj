#!/usr/bin/env python
import rospy
#TODO: Move the end effector in positive-y direction

from joint_controller import PD_Controller_vel

#Call joint_vel_rev_server service to get the reference joint velocities for that end effector velocity
joint_vel_forward = rospy.ServiceProxy('joint_vel_forward', JointVelForward)
joint_vel_rev = rospy.ServiceProxy('joint_vel_rev', JointVelRev)
joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)
current_joint_properties = joint_properties('joint_2')
theta1 = current_joint_properties.position[0]
current_joint_properties = joint_properties('joint_5')
theta2 = current_joint_properties.position[0]
needed_joint_vels = joint_vel_rev(1, 0, 0, theta1, theta2)
PDC = PD_Controller_vel()
#TODO: make velocity controller able to intake the deisred positions
#TODO: Provide those reference joint velocities to the vel_controller so that it provides the required effort
#TODO: To calculate joint velocities, we need jacobian at that position, so we need to be continously recieving the joint positions