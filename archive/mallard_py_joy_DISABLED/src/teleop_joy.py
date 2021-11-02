#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
#  global Coeff
#  if data.buttons[7] == 1:
# 	Coeff=Coeff+0.1
#  if data.buttons[6] == 1:
#     	Coeff=Coeff-0.1
 
 twist = Twist()
 twist.linear.x = 1*data.axes[1]*Coeff
 twist.linear.y = 1*data.axes[0]*Coeff
 twist.angular.z = -0.7*data.axes[3]*Coeff

 twist.angular.x = data.buttons[5]
 twist.angular.y = data.buttons[4]

 pub.publish(twist)

# Intializes everything
def start():
 global Coeff
 Coeff=1
 # publishing to "turtle1/cmd_vel" to control turtle1
 global pub
 pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=2)
 # subscribed to joystick inputs on topic "joy"
 rospy.Subscriber("joy", Joy, callback)
 # starts the node
 rospy.init_node('Joy2Gazebo')
 rospy.spin()

if __name__ == '__main__':
 start()
