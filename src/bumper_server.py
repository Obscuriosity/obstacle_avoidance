#! /usr/bin/env python
# Author: Tony Willett
# Date  : Dydd Gwener trydydd ar ugain o Fai 2025
# Description: Node using actionlib to manage bumper behaviour.
# On receiving call back, do action based on which bumper is hit.

import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class bumperServer:
    '''Node to manage bumper behaviour
    '''
    def __init__(self):
        rospy.loginfo("bumperServer class started")


















if __name__ == '__main__':
    rospy.init_node('bumper_server')
    rospy.loginfo("Bumper server starting")
    bs = bumperServer()
    rospy.spin()