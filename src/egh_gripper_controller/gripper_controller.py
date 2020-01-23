#!/usr/bin/env python
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

from rcomponent.rcomponent import *
from egh_gripper_controller.msg import Status


class GripperController(RComponent):

    def __init__(self):
        RComponent.__init__(self)

        self.status = Status.UNKNOWN
    
    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)
        
        self.status_topic_name = rospy.get_param('~status_topic_name', 'status')
        self.close_service_name = rospy.get_param('~close_service_name', 'close')
        self.open_service_name = rospy.get_param('~open_service_name', 'open')

        self.left_joint_name = rospy.get_param('~left_joint', 'left_joint')
        self.left_joint_open_position = rospy.get_param('~left_joint_open_position', 0.0)
        self.left_joint_close_position = rospy.get_param('~left_joint_close_position', -0.3)
        self.right_joint_name = rospy.get_param('~right_joint', 'right_joint')
        self.right_joint_open_position = rospy.get_param('~right_joint_open_position', 0.0)
        self.right_joint_close_position = rospy.get_param('~right_joint_close_position', -0.3)

    def ros_setup(self):
        """Creates and inits ROS components"""
        RComponent.ros_setup(self)

        self.status_pub = rospy.Publisher('~' + self.status_topic_name, String, queue_size=1)
        self.close_srv = rospy.Service('~' + self.close_service_name, Trigger, self.close_cb)
        self.open_srv = rospy.Service('~' + self.open_service_name, Trigger, self.open_cb)
    
    def ready_state(self):
        self.status_pub.publish(String(self.status))

    def close_cb(self, msg):
        """Close gripper service server callback"""
        raise NotImplementedError("GripperController::close_cb: Should have implemented this" )
    
    def open_cb(self, msg):
        """Open gripper service server callback"""
        raise NotImplementedError("GripperController::close_cb: Should have implemented this" )

