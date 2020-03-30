#!/usr/bin/env python

import rospy
import curses
from geometry_msgs.msg import Twist

def controller():
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(50)
    
    stdcr = curses.initscr()
    stdcr.nodelay(1)
    msg = Twist()

    controls = rospy.get_param('controls')
    
    while not rospy.is_shutdown():
        key = stdcr.getch()
	if key == ord(controls['front']):
            msg.linear.x = 2
        if key == ord(controls['back']):
            msg.linear.x = -2
        if key == ord(controls['left']):
            msg.angular.z = 2
        if key == ord(controls['right']):
            msg.angular.z = -2        
	
	msg.linear.x *= 0.97
	msg.angular.z *= 0.87

	pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
