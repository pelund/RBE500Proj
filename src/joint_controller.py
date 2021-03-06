#!/usr/bin/env python
from __future__ import print_function
import rospy
# import gazebo_msgs.msg
from gazebo_msgs.srv import GetJointProperties
from gazebo_msgs.srv import ApplyJointEffort
import os

#This function will send the joint values with /gazebo/apply_joint_effort
class PD_Controller:
    def __init__(self):
        self.error = 0
        self.old_error = 0
        self.time = rospy.Time.now()
        self.current = 10000000000

    
    def send_joint_efforts(self, effort):
        apply_effort = rospy.ServiceProxy("/gazebo/apply_joint_effort", ApplyJointEffort)
        apply_effort ('joint_6',effort,rospy.Time(),rospy.Duration(0.1))
        #message fields: "joint_name: 'joint2', effort: 10.0, start_time: secs: 0 nsecs: 0,duration: secs: 10 nsecs: 0"

    def PD(self, current, desired):

        catkin_pkg = os.getenv('ROS_PACKAGE_PATH').split(':')
        catkin_pkg = str(catkin_pkg[0])
        self.current = current
        Kp = 30
        # Ku = Kp/0.6
        # Tu = 0.01
        # Kd = (3*Ku*Tu)/40
        Kd = 100

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
        calculated_effort = (Kp * self.error) + (Kd * delta_error) - 9.8 
        print(self.current)
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
    pd_controller = PD_Controller()
    while position_reached == False:
        joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)
        current_joint_properties = joint_properties('joint_6')
        current_joint_position = current_joint_properties.position[0]
        # print(current_joint_position)
        desired_joint_position = 0
        pd_controller.PD(current_joint_position, desired_joint_position)
        if((pd_controller.error > 0.005)and pd_controller.current > desired_joint_position-.005 and pd_controller.current < desired_joint_position+ .005):
            print('Position reached!')
            #break
        else:
            print("Not there")
        
        rospy.sleep(0.1)


    