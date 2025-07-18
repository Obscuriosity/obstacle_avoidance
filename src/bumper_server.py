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

    def __init__(self):
        self._action_name = 'bumpers' # this is the name required by the clients 
        self._as = actionlib.SimpleActionServer(self._action_name, obstacle_avoidance.msg.bumperAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("bumperServer class started: %s", self._action_name)
        self.symud = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        #self.symudol = Twist()
        self.symudol_diwetha = Twist()
        self.bumping = False
        self.bump = rospy.Publisher('bump', Bool, latch=True, queue_size=1)

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(40)
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
            self.symudol = Twist()

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                moving = False
                self.bumping = False
                self.bump.publish(self.bumping)
                break

            percent += 10 # half a second duration at 10Hz
            # check which bumper triggered
            if (goal.bumper_id == 1):
                rospy.loginfo("Front Left.")
                if (percent < 51):
                    self.symudol.linear.x = -reverse
                else:
                    self.symudol.angular.z = -rotate

            if (goal.bumper_id == 2):
                rospy.loginfo("Front Midddle.")
                if (percent < 51):
                    self.symudol.linear.x = -reverse
                else:
                    self.symudol.angular.z = -rotate

            if (goal.bumper_id == 3):
                rospy.loginfo("Front Right.")
                if (percent < 51):
                    self.symudol.linear.x = -reverse
                else:
                    self.symudol.angular.z = rotate

            if (goal.bumper_id == 4):
                rospy.loginfo("Back Left.")
                if (percent < 51):
                    self.symudol.linear.x = reverse
                else:
                    self.symudol.angular.z = rotate

            if (goal.bumper_id == 5):
                rospy.loginfo("Back Middle.")
                if (percent < 51):
                    self.symudol.linear.x = reverse
                else:
                    self.symudol.angular.z = rotate

            if (goal.bumper_id == 6):
                rospy.loginfo("Back Right.")
                if (percent < 51):
                    self.symudol.linear.x = reverse
                else:
                    self.symudol.angular.z = -rotate

            # Publish cmd_vel/Twist
            #rospy.loginfo("Symudol: ")
            #rospy.loginfo(self.symudol.linear.x)
            #rospy.loginfo(self.symudol.angular.z)
            #rospy.loginfo("Symudol Diwetha: ")
            #rospy.loginfo(self.symudol_diwetha.linear.x)
            #rospy.loginfo(self.symudol_diwetha.angular.z)
            if self.symudol != self.symudol_diwetha:
                #rospy.loginfo("Publish CMD_VEL")
                self.symud.publish(self.symudol)
            # Publish Bump Bool
            self.bump.publish(self.bumping)

            self._feedback.percent_complete = percent
            self._as.publish_feedback(self._feedback)
            if (percent >= 100):
                moving = False
            #r.sleep()

            self.symudol_diwetha = self.symudol
            #rospy.loginfo("Loop done, update Symudol_diwetha")
            #rospy.loginfo("Symudol Diwetha: ")
            #rospy.loginfo(self.symudol_diwetha.linear.x)
            #rospy.loginfo(self.symudol_diwetha.angular.z)
            #rospy.loginfo("-")
            r.sleep()

        if success:
            self._result.Done = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
            self.symudol.angular.z = 0.0
            self.symudol.linear.x = 0.0
            self.symud.publish(self.symudol)
            self.bumping = False
            self.bump.publish(self.bumping)
            self.symudol_diwetha = self.symudol
            


if __name__ == '__main__':
    rospy.init_node('bumpers')
    rospy.loginfo("Bumper server starting")
    bs = bumperServer()
    rospy.spin()