#!/usr/bin/env python
import rospy
# import numpy as np
# from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def set_wheel_vel(a, b, c, d):
    wheel_vel.linear.x = a  #Wheel A. Top Left
    wheel_vel.linear.y = b  #Wheel B. Top Right
    wheel_vel.linear.z = c  #Wheel C. Bottom Left
    wheel_vel.angular.x = d #Wheel D. Bottom Right


def stop_forawhile(t):
    for j in range(t):
        pub.publish(Twist())
        rate.sleep()


def move():
    global rate, pub, wheel_vel

    rate = rospy.Rate(1) # 100Hz = 0.01s

    pub = rospy.Publisher("/mallard/cmd_wheel", Twist, queue_size=1)

    wheel_vel = Twist()

    # for-loop because publishing in topics sometimes fails the first time of publishing
    for i in range(10):
        connections = pub.get_num_connections()
        if connections > 0:
            pub.publish(wheel_vel)
            rospy.loginfo("wheel_vel Published")
            break
        else:
            rate.sleep()

    # for i in range(10):
    #     # delay for pi (3.14) seconds
    #     for j in range(628):
    #         pub.publish(wheel_vel)
    #         rate.sleep()
        
    #     for j in range(100):
    #         pub.publish(Twist())
    #         rate.sleep()


    for j in range(3):
        set_wheel_vel(5, -5, 5, -5)
        pub.publish(wheel_vel)
        rate.sleep()

    stop_forawhile(1)

    for j in range(3):
        set_wheel_vel(-5, 5, -5, 5)
        pub.publish(wheel_vel)
        rate.sleep()

    stop_forawhile(1)

    for j in range(3):
        set_wheel_vel(-5, -5, 5, 5)
        pub.publish(wheel_vel)
        rate.sleep()

    stop_forawhile(1)

    for j in range(3):
        set_wheel_vel(5, 5, -5, -5)
        pub.publish(wheel_vel)
        rate.sleep()

    stop_forawhile(1)

    for j in range(10):
        set_wheel_vel(-2.1, -2.1, -2.1, -2.1)
        pub.publish(wheel_vel)
        rate.sleep()

    stop_forawhile(1)

    for j in range(10):
        set_wheel_vel(2.1, 2.1, 2.1, 2.1)
        pub.publish(wheel_vel)
        rate.sleep()


    #Stop
    pub.publish(Twist())
    
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('test_wheel_vel_El_move')
    move()

