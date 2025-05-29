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
        self._ac.wait_for_server()

        # Subscribers:
        self.frontL = rospy.Subscriber('bpr_lf', Bool, self.bumperLF_CB)
        self.frontM = rospy.Subscriber('bpr_mf', Bool, self.bumper_CB)
        self.frontR = rospy.Subscriber('bpr_rf', Bool, self.bumper_CB)
        self.backL = rospy.Subscriber('bpr_lb', Bool, self.bumper_CB)
        self.backM = rospy.Subscriber('bpr_mb', Bool, self.bumper_CB)
        self.backR = rospy.Subscriber('bpr_rb', Bool, self.bumper_CB)

        rospy.loginfo("bumperClient started")

    def bumperLF_CB(self, bump):
        if (bump.data):
            rospy.loginfo("Bumper Hit")
            goal = obstacle_avoidance.msg.bumperGoal(bumper_id=1)
            self._ac.send_goal(goal)
            self._ac.wait_for_result()
            self._ac.get_result()
            #rospy.loginfo('Result: ', )

    def bumper_CB(self, bump):
        if (bump.data):
            rospy.loginfo("Bumper Hit")
        





if __name__ == '__main__':
    rospy.init_node('bumpers_client')
    rospy.loginfo("Bumper client starting")
    bc = bumperClient()
    rospy.spin()