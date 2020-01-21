#!/usr/bin/env python
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

from ur_msgs.srv import SetIO, SetIORequest

from rcomponent.rcomponent import *
from fake_joint_position_publisher.srv import SetJoints, SetJointsRequest
from egh_gripper_controller.msg import Status


class URIOGripperController(RComponent):

    def __init__(self):
        RComponent.__init__(self)

        self.status = Status.UNKNOWN
    
    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)
        
        self.status_topic_name = rospy.get_param('~status_topic_name', 'status')
        self.close_service_name = rospy.get_param('~close_service_name', 'close')
        self.open_service_name = rospy.get_param('~open_service_name', 'open')

        self.ur_io_service_name = rospy.get_param('~ur_io_service_name', 'ur_driver/set_io')
        self.update_joints_service_name = rospy.get_param('~update_joints_service_name', 'joints_updater/set_joint')

        self.left_joint_name = rospy.get_param('~left_joint', 'left_joint')
        self.left_joint_open_position = rospy.get_param('~left_joint_open_position', 0.0)
        self.left_joint_close_position = rospy.get_param('~left_joint_close_position', -0.3)
        self.right_joint_name = rospy.get_param('~right_joint', 'right_joint')
        self.right_joint_open_position = rospy.get_param('~right_joint_open_position', 0.0)
        self.right_joint_close_position = rospy.get_param('~right_joint_close_position', -0.3)

        self.open_request = SetJointsRequest()
        self.open_request.joints_names = [self.left_joint_name, self.right_joint_name]
        self.open_request.positions = [self.left_joint_open_position, self.right_joint_open_position]

        self.close_request = SetJointsRequest()
        self.close_request.joints_names = [self.left_joint_name, self.right_joint_name]
        self.close_request.positions = [self.left_joint_close_position, self.right_joint_close_position]

        self.set_io_request = SetIORequest()
        self.set_io_request.fun = 1
        self.set_io_request.state = 1

    def ros_setup(self):
        """Creates and inits ROS components"""
        RComponent.ros_setup(self)

        self.status_pub = rospy.Publisher('~' + self.status_topic_name, String, queue_size=1)
        self.close_srv = rospy.Service('~' + self.close_service_name, Trigger, self.close_cb)
        self.open_srv = rospy.Service('~' + self.open_service_name, Trigger, self.open_cb)

        self.ur_io_client = rospy.ServiceProxy(self.ur_io_service_name, SetIO)
        self.update_joints_client = rospy.ServiceProxy(self.update_joints_service_name, SetJoints)
    
    def ready_state(self):
        self.status_pub.publish(String(self.status))

    def close_cb(self, msg):
        """Close gripper service server callback"""
        response = TriggerResponse()
        response.message = "Gripper correctly closed"
        
        self.set_io_request.pin = 5
        set_io_res = self.ur_io_client.call(self.set_io_request)
        
        if set_io_res.success == True:
            update_joints_res = self.update_joints_client.call(self.close_request)
            if update_joints_res.success == True:
                response.success = True
            else:
                response.message = "Failed updating joints positions"
        else:
            response.message = "Failed setting UR output"
        
        self.status = Status.CLOSED
        print response
        return response
    
    def open_cb(self, msg):
        """Open gripper service server callback"""
        response = TriggerResponse()
        response.message = "Gripper correctly opened"
        
        self.set_io_request.pin = 4
        set_io_res = self.ur_io_client.call(self.set_io_request)
        
        if set_io_res.success == True:
            update_joints_res = self.update_joints_client.call(self.open_request)
            if update_joints_res.success == True:
                response.success = True
            else:
                response.message = "Failed updating joints positions"
        else:
            response.message = "Failed setting UR output"
        
        self.status = Status.OPENED
        return response
