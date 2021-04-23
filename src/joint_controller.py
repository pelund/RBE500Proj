import rospy
import gazebo_msgs
#Gazebo_msg documentation:http://docs.ros.org/en/jade/api/gazebo_msgs/html/index-msg.html
#Docs were hard to find, were not on wiki but somewhere else
from gazebo_msgs.srv import GetJointProperties
from gazebo_msgs.srv import ApplyJointEffort


#This function will send the joint values with /gazebo/apply_joint_effort
class PD_Controller:
    def __init__(self):
        self.error = 3
        self.old_error = 0
    
    def send_joint_efforts(self, effort):
        apply_effort = rospy.ServiceProxy("/gazebo/apply_joint_effort", ApplyJointEffort)
        apply_effort ('joint_6',effort,rospy.Time(),rospy.Duration(1))
        #message fields: "joint_name: 'joint2', effort: 10.0, start_time: secs: 0 nsecs: 0,duration: secs: 10 nsecs: 0"

    def PD(self, current, desired):
        self.error = desired - current
        delta_error = self.old_error - self.error
        calculated_effort = (0.6 * self.error) + (8 * delta_error)
        print(self.error)
        self.send_joint_efforts(calculated_effort)
        self.old_error = self.error

#must implement a service to get a reference position for the last joint and get it to go there

#This part will obtain the joint positions from gazebo
#should be a subscriber

if __name__ == '__main__':
    rospy.init_node('joint_controller')
    position_reached = False
    pd_controller = PD_Controller()
    while position_reached == False:
        joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)
        current_joint_properties = joint_properties('joint_6')
        current_joint_position = current_joint_properties.position[0]
        # print(current_joint_position)
        desired_joint_position = -0.5

        pd_controller.PD(current_joint_position, desired_joint_position)
        if(pd_controller.error > 0.005):
            print('Position reached!')
            break
        rospy.sleep(0.1)


    