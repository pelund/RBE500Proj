#!/usr/bin/env python
from __future__ import print_function
import rospy
# import gazebo_msgs.msg
from gazebo_msgs.srv import GetJointProperties
from gazebo_msgs.srv import ApplyJointEffort
import os

#This function will send the joint values with /gazebo/apply_joint_effort
class PD_Controller:
    def __init__(self, joint_name, Kp, Kd):
        self.error = 0
        self.old_error = 0
        self.time = rospy.Time.now()
        self.current = 10000000000
        self.joint_name = joint_name
        self.Kp = Kp 
        self.Kd = Kd

    def send_joint_efforts(self, effort):
        apply_effort = rospy.ServiceProxy("/gazebo/apply_joint_effort", ApplyJointEffort)
        apply_effort (self.joint_name, effort, rospy.Time(),rospy.Duration(0.1))
        #message fields: "joint_name: 'joint2', effort: 10.0, start_time: secs: 0 nsecs: 0,duration: secs: 10 nsecs: 0"

    def PD(self, current, desired):

        catkin_pkg = os.getenv('ROS_PACKAGE_PATH').split(':')
        catkin_pkg = str(catkin_pkg[0])
        self.current = current
        # Ku = Kp/0.6
        # Tu = 0.01
        # Kd = (3*Ku*Tu)/40

        #Without gravity
        # Kp = 25
        # Kd = 10
        self.error = (desired - current)

        desired_file = open(catkin_pkg + "/rbe_proj/src/desired.txt", "a")
        desired_file.write(str(desired) + '\n')
        desired_file.close()

        current_file =  desired_file = open(catkin_pkg + "/rbe_proj/src/current.txt", "a")
        current_file.write(str(current) + '\n')
        current_file.close()

        delta_error = self.error- self.old_error 
        if(self.joint_name == 'joint_6'):
            calculated_effort = (self.Kp * self.error) + (self.Kd * delta_error) - 9.8 
        else:
            calculated_effort = (self.Kp * self.error) + (self.Kd * delta_error) 

        
        print('Joint :' + str(self.joint_name) + 'at ' + str(self.current))
        self.send_joint_efforts(calculated_effort)
        self.old_error = self.error
        # self.osc_per = rospy.Time.now() - self.time

#must implement a service to get a reference position for the last joint and get it to go there

#This part will obtain the joint positions from gazebo
#should be a subscriber

if __name__ == '__main__':
    catkin_pkg = os.getenv('ROS_PACKAGE_PATH').split(':')
    catkin_pkg = str(catkin_pkg[0])
    desired_file = open(catkin_pkg + "/rbe_proj/src/desired.txt", "r+")
    current_file = open(catkin_pkg + "/rbe_proj/src/current.txt", "r+")
    
    desired_file.truncate(0)
    desired_file.close()
    current_file.truncate(0)
    current_file.close()

    rospy.init_node('joint_controller')
    position_reached = False
    pd_controller1 = PD_Controller('joint_2', 10, 120)
    pd_controller2 = PD_Controller('joint_5', 10, 40)
    pd_controller3 = PD_Controller('joint_6', 15, 20)

    while position_reached == False:
        joint_2properties = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)
        joint_5properties = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)
        joint_6properties = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)

        current_joint_2properties = joint_2properties('joint_2')
        current_joint_5properties = joint_5properties('joint_5')
        current_joint_6properties = joint_6properties('joint_6')

        current_joint_2position = current_joint_2properties.position[0]
        current_joint_5position = current_joint_5properties.position[0]
        current_joint_6position = current_joint_6properties.position[0]
        # print(current_joint_position)
        desired_joint_2position = 0.8
        desired_joint_5position = 0.8
        desired_joint_6position = 0.2

        pd_controller1.PD(current_joint_2position, desired_joint_2position)
        pd_controller2.PD(current_joint_5position, desired_joint_5position)
        pd_controller3.PD(current_joint_6position, desired_joint_6position)
        
        if((pd_controller2.error > 0.005)and pd_controller2.current > desired_joint_5position-.005 and pd_controller2.current < desired_joint_5position+ .005):
            print('Position reached!')
            #break
        else:
            print("Not there")
        
        rospy.sleep(0.1)


    