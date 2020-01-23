#!/usr/bin/env python
from std_msgs.msg import Float64

from gripper_controller import *

class GazeboGripperController(GripperController):

    def __init__(self):
        GripperController.__init__(self)
    
    def ros_read_params(self):
        """Gets params from param server"""
        GripperController.ros_read_params(self)
        
        self.finger_left_topic = rospy.get_param('~finger_left_topic', 'joint_left_finger_controller/command')
        self.finger_right_topic = rospy.get_param('~finger_right_topic', 'joint_right_finger_controller/command')

    def ros_setup(self):
        """Creates and inits ROS components"""
        GripperController.ros_setup(self)

        self.finger_left_publisher = rospy.Publisher(self.finger_left_topic, Float64, queue_size=1)
        self.finger_right_publisher = rospy.Publisher(self.finger_right_topic, Float64, queue_size=1)
    
    def ready_state(self):
        GripperController.ready_state(self)

    def close_cb(self, msg):
        """Close gripper service server callback"""
        command = Float64()
        command.data = self.left_joint_close_position
        self.finger_left_publisher.publish(command)
        command.data = self.right_joint_close_position
        self.finger_right_publisher.publish(command)

        response = TriggerResponse()
        response.success = True       
        response.message = "Gripper correctly closed"       
        self.status = Status.CLOSED
        return response
    
    def open_cb(self, msg):
        """Open gripper service server callback"""
        command = Float64()
        command.data = self.left_joint_open_position
        self.finger_left_publisher.publish(command)
        command.data = self.right_joint_open_position
        self.finger_right_publisher.publish(command)

        response = TriggerResponse()
        response.message = "Gripper correctly closed"       
        response.success = True       
        self.status = Status.OPENED
        return response
