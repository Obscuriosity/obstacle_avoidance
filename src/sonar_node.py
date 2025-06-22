#! /usr/bin/env python3
# Author: Tony Willett
# Date  : Dydd Iau deuddeg o Mehefin 2025
# Description: Braitenberg vehicles style sonar reactions, speed and rotation are relative to object distance,
# as objects get closer, rotation increases and speed decreases.

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class Sonar(object):

    '''
    Sonar class for motion control and obstacle avoidance.
    '''

    def __init__(self):
        self.linear = 0.5
        self.angular = 0.0
        self.move = True
        self.sonarL = 0
        self.sonarLM = 0
        self.sonarRM = 0
        self.sonarR = 0
        self.symydol = Twist()

        # Subscribers
        # subscribe to bump state, only move when bump is false
        rospy.Subscriber('bump', Bool, self.bump_CB)
        # subscribe to sonar
        rospy.Subscriber('snr_1', Range, self.snr_1_CB)
        rospy.Subscriber('snr_2', Range, self.snr_2_CB)
        rospy.Subscriber('snr_3', Range, self.snr_3_CB)
        rospy.Subscriber('snr_4', Range, self.snr_4_CB)

        # Publishers
        # publish cmd_vel for base controller
        self.symud = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Call Backs
    def bump_CB(self, bump):
        rospy.loginfo(bump.data)
        self.move = not bump.data
        rospy.loginfo("move - %s", self.move)
    
    def snr_1_CB(self, distance):
        self.sonarL = distance.range
    
    def snr_2_CB(self, distance):
        self.sonarLM = distance.range
    
    def snr_3_CB(self, distance):
        self.sonarRM = distance.range
    
    def snr_4_CB(self, distance):
        self.sonarR = distance.range

        
    # Methods
    def avoid(self):
        # put this in a loop
        if (self.move):
            rospy.loginfo("Moving!")
            # speed and rotation depend on sonar readings
            # the closer the object the lower the speed and higher the rotation

            # get distances - deal with zeros



def main():
    while not rospy.is_shutdown():
        sn = Sonar()
        sn.avoid()
    # rospy.spin()


if __name__ == '__main__':
    rospy.init_node('sonar_node')
    rospy.loginfo("Sonar node starting")
    #sn = Sonar()
    #sn.avoid()
    #rospy.spin()