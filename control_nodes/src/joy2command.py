#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def joy2cmd_callback(joyData):
    # Receive joystick messages (subscribed to joy topic)
    cmd_vel.linear.x = joyData.axes[1]  # left stick Vertical
    cmd_vel.linear.y = joyData.axes[0]  # left stick Horizontal
    cmd_vel.angular.z = joy_gain*joyData.axes[3] # right stick Horizontal

    cmd_vel.linear.z = joyData.buttons[0]  # X
    cmd_vel.angular.x = joyData.buttons[5] # R1
    cmd_vel.angular.y = joyData.buttons[4] # L1

    pub.publish(cmd_vel)

def joy2cmd():
    global cmd_vel
    global joy_gain
    global pub

    cmd_vel = Twist()

    joy_gain = 1
    
    rospy.Subscriber("/joy", Joy, joy2cmd_callback)
    pub = rospy.Publisher("/mallard/cmd_vel", Twist, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('Joy2cmd')
    joy2cmd()
    rospy.spin()