#!/usr/bin/env python
# Authors: TriKnight

import rospy
from geometry_msgs.msg import Twist
import time 

LINEAR_X_VEL = 0.5
LINEAR_Y_VEL = 0.5


class Moving():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._pub_rate =0.1
            
    def go_forward(self, duration, speed):

        # Get the initial time
        self.t_init = time.time()
        # Set the velocity forward and wait (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not rospy.is_shutdown():

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self._cmd_pub.publish(msg)
            time.sleep(self._pub_rate)

    def go_side(self, duration, speed):
        self.t_init = time.time()
        while time.time() - self.t_init < duration and not rospy.is_shutdown():

            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = speed
            msg.angular.z = 0
            self._cmd_pub.publish(msg)
            time.sleep(self._pub_rate)

    def stop(self):
        self.t_init = time.time()
        while time.time() - self.t_init < 0.1 and not rospy.is_shutdown():
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.angular.z = 0
            self._cmd_pub.publish(msg)
            time.sleep(self._pub_rate)

def main():
    rospy.init_node('cleaner_move')
    try:
        robotmove = Moving()
        robotmove.go_forward(3.0, LINEAR_X_VEL)
        robotmove.go_side(3.0, LINEAR_Y_VEL)
        robotmove.stop()
        robotmove.go_forward(6.0, LINEAR_X_VEL)
        robotmove.stop()
 
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()