#! /usr/bin/env python3
# Author: Tony Willett
# Date  : Dydd Iau nawfed ar ugain o Fai 2025
# Description: Node to listen subscribe to bumper hits and calls bumper action client to react.

import rospy
import math
import actionlib
import obstacle_avoidance.msg


class bumperClient:
    '''Bumper Client Class'''
    
    def __init__(self):

        self._ac = actionlib.SimpleActionClient('bumpers', obstacle_avoidance.msg.bumperAction)
        self._ac.start()
        self._ac.wait_for_server()

        # Subscribers:
        self.frontL = rospy.Subscriber('bpr_lf', Bool, self.bumper_CB)
        self.frontM = rospy.Subscriber('bpr_mf', Bool, self.bumper_CB)
        self.frontR = rospy.Subscriber('bpr_lr', Bool, self.bumper_CB)
        self.BackL = rospy.Subscriber('bpr_lb', Bool, self.bumper_CB)
        self.BackL = rospy.Subscriber('bpr_mb', Bool, self.bumper_CB)
        self.BackL = rospy.Subscriber('bpr_rbf', Bool, self.bumper_CB)

    def bumper_CB(self, bump):
        rospy.loginfo("Bumper Hit")
        





if __name__ == '__main__':
    rospy.init_node('bumper_client')
    rospy.loginfo("Bumper client starting")
    bc = bumperClient()
    rospy.spin()