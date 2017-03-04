#!/usr/bin/env python
import roslib; 
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
a  s  d  f

Hold Shift to make it faster 
"""

moveBindings = {
		'w':(0.5,0,0,0),
		'a':(0,0,0,-5),
		'd':(0,0,0,5),
		's':(-0.5,0,0,0),
		'W':(1,0,0,0),
		'A':(0,0,0,-10),
		'D':(0,0,0,10),
		'S':(-1,0,0,0),
	       }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = 1
turn = 1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('snoopy/cmd_vel', Twist, queue_size = 1)
	rospy.init_node('teleop_twist_keyboard')

	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x; twist.linear.y = y; twist.linear.z = z;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
			pub.publish(twist)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


