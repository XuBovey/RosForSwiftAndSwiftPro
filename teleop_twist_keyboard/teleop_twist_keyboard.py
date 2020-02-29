#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Char,Bool

import sys, select, termios, tty

msg = """
Reading from the keyboard and Publishing to my_cmd_vel and sunction_cup_ctrl!
------------------------
Moving motor
  motor1 motor2 motor3 motor4
+ u	    i	    o      p
- j	    k	    l      ;

q/z: increase/decrease speed

w/s: suctionCup Enable/Disable

CTRL-C to quit
"""
moveBindings = {
        'u':(1,0,0,0),
        'i':(0,1,0,0),
        'o':(0,0,1,0),
        'p':(0,0,0,1),
        'j':(-1,0,0,0),
        'k':(0,-1,0,0),
        'l':(0,0,-1,0),
        ';':(0,0,0,-1),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('my_cmd_vel', Twist, queue_size = 1)
    pub_cupVel = rospy.Publisher('sunction_cup_ctrl', Char, queue_size = 1)
    pub_sim_start = rospy.Publisher('startSimulation', Bool, queue_size = 1)
    pub_sim_pause = rospy.Publisher('pauseSimulation', Bool, queue_size = 1)
    pub_sim_stop = rospy.Publisher('stopSimulation', Bool, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    cupVel = False
    status = 0
    twistEn = 0
    cupVelEn = 0
    vrep_simulation_state = 0 # 0- stop, 1- start, 2- pause

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
	    key = getKey()
	    if key == ' ': # space -- pause
		if vrep_simulation_state == 0:
		    vrep_simulation_state = 1 # start
		    pub_sim_start.publish(True)
		elif vrep_simulation_state == 1:
		    vrep_simulation_state = 2 # pause
		    pub_sim_pause.publish(True)
		else:
		    vrep_simulation_state = 1
		    pub_sim_start.publish(True)
	    elif key == '\x1B': # esc -- stop simultion
		vrep_simulation_state = 0
		pub_sim_stop.publish(True)

            elif key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
		twistEn = 1
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == 'w':
                cupVel = 1
		pub_cupVel.publish(cupVel)
            elif key == 's':
                cupVel = 0
		pub_cupVel.publish(cupVel)
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

	    if twistEn == 1:
		twistEn = 0
            	twist = Twist()
            	twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            	pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
	pub_sim_stop.publish(True)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
