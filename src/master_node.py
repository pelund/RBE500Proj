#!/usr/bin/env python

#TODO: Move the end effector in positive-y direction
#TODO: Call joint_vel_rev_server service to get the reference joint velocities for that end effector velocity
#TODO: Provide those reference joint velocities to the vel_controller so that it provides the required effort
#TODO: To calculate joint velocities, we need jacobian at that position, so we need to be continously recieving the joint positions