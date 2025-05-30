#! /usr/bin/env python3
# Author: Tony Willett
# Date  : Dydd Gwener trydydd ar ugain o Fai 2025
# Description: Node using actionlib to manage bumper behaviour.
# On receiving call back, do action based on which bumper is hit.

import rospy
import math
import actionlib
import obstacle_avoidance.msg
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class bumperServer(object):
    '''Node to manage bumper behaviour
    subscribes via the client to bumper switches
    publishes cmd_vel
    '''
    # create messages that are used to publish feedback/result
    _feedback = obstacle_avoidance.msg.bumperFeedback()
    _result = obstacle_avoidance.msg.bumperResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, obstacle_avoidance.msg.bumperAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("bumperServer class started")
        self.symud = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(10)
        success = True
        moving = True
        percent = 0
        
        # print progress
        rospy.loginfo("bumper action started: %s" % goal)
        rospy.loginfo(goal)

        #execute the action
        while moving:
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            percent += 20 # half a second duration at 10Hz
            symudol = Twist()
            # check which bumper triggered
            if (goal.bumper_id == 1):
                rospy.loginfo("Front Left.")
                symudol.angular.z = -0.4
                symudol.linear.x = 0.0

            if (goal.bumper_id == 2):
                rospy.loginfo("Front Midddle.")
                symudol.angular.z = -0.4
                symudol.linear.x = 0.0

            if (goal.bumper_id == 3):
                rospy.loginfo("Front Right.")
                symudol.angular.z = 0.4
                symudol.linear.x = 0.0

            if (goal.bumper_id == 4):
                rospy.loginfo("Back Left.")
                symudol.angular.z = 0.4
                symudol.linear.x = 0.0

            if (goal.bumper_id == 5):
                rospy.loginfo("Back Middle.")
                symudol.angular.z = 0.4
                symudol.linear.x = 0.0

            if (goal.bumper_id == 6):
                rospy.loginfo("Back Right.")
                symudol.angular.z = -0.4
                symudol.linear.x = 0.0

            # Publish cmd_vel/Twist
            self.symud.publish(symudol)

            self._feedback.percent_complete = percent
            self._as.publish_feedback(self._feedback)
            if (percent >= 100):
                moving = False
            # for testing, remove later
            r.sleep()

        if success:
            self._result.Done = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
            symudol.angular.z = 0.0
            self.symud.publish(symudol)


if __name__ == '__main__':
    rospy.init_node('bumpers')
    rospy.loginfo("Bumper server starting")
    bs = bumperServer(rospy.get_name())
    rospy.spin()