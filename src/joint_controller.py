import rospy
import gazebo_msgs
#Gazebo_msg documentation:http://docs.ros.org/en/jade/api/gazebo_msgs/html/index-msg.html
#Docs were hard to find, were not on wiki but somewhere else


    

#This function will send the joint values with /gazebo/apply_joint_effort
def send_joint_efforts(effort):
    apply_effort = rospy.ServiceProxy("/gazebo/apply_joint_effort", ApplyJointEffort)
    apply_effort (joint6,effort,rospy.Time(),rospy.Duration(1))
    #message fields: "joint_name: 'joint2', effort: 10.0, start_time: secs: 0 nsecs: 0,duration: secs: 10 nsecs: 0"

def PD(current,desired):
    global old_error = error
    global error = desired - current
    delta_error = old_error - error
    calculated_effort = kp * error + kd * delta_error
    send_joint_efforts(calculated_effort)



#must implement a service to get a reference position for the last joint and get it to go there

#
#This part will obtain the joint positions from gazebo
#should be a subscriber
rospy.init_node('joint_controller')
global old_error = 100000
global error = 100000
position_reached = False
while position_reached == False:
    joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)
    current_joint_properties =joint_properties('joint_6')
    current_joint_position = current_joint_properties.position
    desired_joint_position = void #response from goal point service will go here
    if current_joint_position == desired_joint_position:
        position_reached == True
    else
        PD(current_joint_position, desired_joint_position)
rospy.spin()