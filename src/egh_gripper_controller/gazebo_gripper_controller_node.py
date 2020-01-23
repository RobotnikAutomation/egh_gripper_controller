#!/usr/bin/env python
import rospy

from gazebo_gripper_controller import GazeboGripperController

def main():
    rospy.init_node("GazeboGripperController")
    publisher = GazeboGripperController()
    rospy.loginfo('%s: starting'%(rospy.get_name()))
    publisher.start()

if __name__ == "__main__":
    main()
    exit()