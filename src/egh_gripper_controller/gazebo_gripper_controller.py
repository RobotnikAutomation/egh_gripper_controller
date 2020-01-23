#!/usr/bin/env python
from std_msgs.msg import Float64

from gripper_controller import *

class GazeboGripperController(GripperController):

    def __init__(self):
        GripperController.__init__(self)
    
    def ros_read_params(self):
        """Gets params from param server"""
        GripperController.ros_read_params(self)
        
        self.joint_left_topic = rospy.get_param('~joint_left_topic', 'joint_left_finger_controller/command')
        self.joint_right_topic = rospy.get_param('~joint_right_topic', 'joint_right_finger_controller/command')

    def ros_setup(self):
        """Creates and inits ROS components"""
        GripperController.ros_setup(self)

        self.joint_left_publisher = rospy.Publisher(self.joint_left_topic, Float64, queue_size=1)
        self.joint_right_publisher = rospy.Publisher(self.joint_right_topic, Float64, queue_size=1)
    
    def ready_state(self):
        GripperController.ready_state(self)

    def close_cb(self, msg):
        """Close gripper service server callback"""
        self.left_joint_desired_position = self.left_joint_close_position
        self.right_joint_desired_position = self.right_joint_close_position
        self.write_reference()

        response = TriggerResponse()
        response.success = True       
        response.message = "Gripper correctly closed"       
        self.status = Status.CLOSED
        return response
    
    def open_cb(self, msg):
        """Open gripper service server callback"""
        self.left_joint_desired_position = self.left_joint_open_position
        self.right_joint_desired_position = self.right_joint_open_position
        self.write_reference()

        response = TriggerResponse()
        response.message = "Gripper correctly open"       
        response.success = True       
        self.status = Status.OPENED
        return response

    def write_reference(self):
        command = Float64()
        command.data = self.left_joint_desired_position
        self.joint_left_publisher.publish(command)
        command.data = self.right_joint_desired_position
        self.joint_right_publisher.publish(command)
        