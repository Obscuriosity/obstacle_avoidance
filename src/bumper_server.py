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
        self.bumping = False
        self.bump = rospy.Publisher('Bump', Bool, queue_size=1)

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(10)
        success = True
        moving = True
        percent = 0
        reverse = 0.99
        rotate = 0.4
        self.bumping = True

        
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
                self.bumping = False
                self.bump.publish(self.bumping)
                break
            percent += 10 # half a second duration at 10Hz
            symudol = Twist()
            # check which bumper triggered
            if (goal.bumper_id == 1):
                rospy.loginfo("Front Left.")
                if (percent < 51):
                    symudol.linear.x = -reverse
                else:
                    symudol.angular.z = -rotate

            if (goal.bumper_id == 2):
                rospy.loginfo("Front Midddle.")
                if (percent < 51):
                    symudol.linear.x = -reverse
                else:
                    symudol.angular.z = -rotate

            if (goal.bumper_id == 3):
                rospy.loginfo("Front Right.")
                if (percent < 51):
                    symudol.linear.x = -reverse
                else:
                    symudol.angular.z = rotate

            if (goal.bumper_id == 4):
                rospy.loginfo("Back Left.")
                if (percent < 51):
                    symudol.linear.x = reverse
                else:
                    symudol.angular.z = rotate

            if (goal.bumper_id == 5):
                rospy.loginfo("Back Middle.")
                if (percent < 51):
                    symudol.linear.x = reverse
                else:
                    symudol.angular.z = rotate

            if (goal.bumper_id == 6):
                rospy.loginfo("Back Right.")
                if (percent < 51):
                    symudol.linear.x = reverse
                else:
                    symudol.angular.z = -rotate

            # Publish cmd_vel/Twist
            self.symud.publish(symudol)
            # Publish Bump Bool
            self.bump.publish(self.bumping)

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
            symudol.linear.x = 0.0
            self.symud.publish(symudol)
            self.bumping = False
            self.bump.publish(self.bumping)
            


if __name__ == '__main__':
    rospy.init_node('bumpers')
    rospy.loginfo("Bumper server starting")
    bs = bumperServer(rospy.get_name())
    rospy.spin()