#! /usr/bin/env python3
# Author: Tony Willett
# Date  : Dydd Iau nawfed ar ugain o Fai 2025
# Description: Node to listen subscribe to bumper hits and calls bumper action client to react.

import rospy
import actionlib
import obstacle_avoidance.msg
from std_msgs.msg import Bool

class bumperClient:
    '''Bumper Client Class'''
    
    def __init__(self):

        self._ac = actionlib.SimpleActionClient('bumpers', obstacle_avoidance.msg.bumperAction)
        rospy.loginfo("bumperClient waiting for server")
        self._ac.wait_for_server()
        rospy.loginfo("bumperClient subscribing to bumper topics")

        # Subscribers:
        self.frontL = rospy.Subscriber('bpr_lf', Bool, self.bumperLF_CB)
        self.frontM = rospy.Subscriber('bpr_mf', Bool, self.bumperMF_CB)
        self.frontR = rospy.Subscriber('bpr_rf', Bool, self.bumperRF_CB)
        self.backL = rospy.Subscriber('bpr_lb', Bool, self.bumperLB_CB)
        self.backM = rospy.Subscriber('bpr_mb', Bool, self.bumperMB_CB)
        self.backR = rospy.Subscriber('bpr_rb', Bool, self.bumperRB_CB)

        rospy.loginfo("bumperClient started")

    def bumperLF_CB(self, bump):
        if (bump.data):
            bpr_id = 1
            rospy.loginfo("Bumper %s Hit" % bpr_id)
            goal = obstacle_avoidance.msg.bumperGoal(bumper_id=bpr_id)
            self._ac.send_goal(goal)
            self._ac.wait_for_result()
            self._ac.get_result()

    def bumperMF_CB(self, bump):
        if (bump.data):
            bpr_id = 2
            rospy.loginfo("Bumper %s Hit" % bpr_id)
            goal = obstacle_avoidance.msg.bumperGoal(bumper_id=bpr_id)
            self._ac.send_goal(goal)
            self._ac.wait_for_result()
            self._ac.get_result()
        
    def bumperRF_CB(self, bump):
        if (bump.data):
            bpr_id = 3
            rospy.loginfo("Bumper %s Hit" % bpr_id)
            goal = obstacle_avoidance.msg.bumperGoal(bumper_id=bpr_id)
            self._ac.send_goal(goal)
            self._ac.wait_for_result()
            self._ac.get_result()
    
    def bumperLB_CB(self, bump):
        if (bump.data):
            bpr_id = 4
            rospy.loginfo("Bumper %s Hit" % bpr_id)
            goal = obstacle_avoidance.msg.bumperGoal(bumper_id=bpr_id)
            self._ac.send_goal(goal)
            self._ac.wait_for_result()
            self._ac.get_result()

    def bumperMB_CB(self, bump):
        if (bump.data):
            bpr_id = 5
            rospy.loginfo("Bumper %s Hit" % bpr_id)
            goal = obstacle_avoidance.msg.bumperGoal(bumper_id=bpr_id)
            self._ac.send_goal(goal)
            self._ac.wait_for_result()
            self._ac.get_result()

    def bumperRB_CB(self, bump):
        if (bump.data):
            bpr_id = 6
            rospy.loginfo("Bumper %s Hit" % bpr_id)
            goal = obstacle_avoidance.msg.bumperGoal(bumper_id=bpr_id)
            self._ac.send_goal(goal)
            self._ac.wait_for_result()
            self._ac.get_result()


if __name__ == '__main__':
    rospy.init_node('bumpers_client')
    rospy.loginfo("Bumper client starting")
    bc = bumperClient()
    rospy.spin()