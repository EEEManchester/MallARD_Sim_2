#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist

#Function that transforms body frame velocities in x y and angular to wheel angular velocities
def transform_velocities(V_x, V_y, W_z):
	wheel_velocity[0]=(V_x-V_y+(lx+ly)*W_z)/R
	wheel_velocity[1]=-(V_x+V_y-(lx+ly)*W_z)/R
	wheel_velocity[2]=(V_x+V_y+(lx+ly)*W_z)/R
	wheel_velocity[3]=-(V_x-V_y-(lx+ly)*W_z)/R
	return wheel_velocity

#Function that takes forces in x and y and torque in the body frame and transforms them to
#forces that the wheels should exert 
def transform_forces(Fx, Fy, Tz):
	wheel_forces=[0,0,0,0]
	wheel_forces[0]=Fx*0.353553391-Fy*0.353553391+Tz*1.178511302
	wheel_forces[1]=-(Fx*0.353553391+Fy*0.353553391-Tz*1.178511)		
	wheel_forces[2]=Fx*0.353553391+Fy*0.353553391+Tz*1.178511
	wheel_forces[3]=-(Fx*0.353553391-Fy*0.353553391-Tz*1.178511)
	return wheel_forces	

def callback(twist):
	#A list that will hold the values for the wheel angular velocities/wheel forces for mode 0
	global wheel_velocity
	#Variables that hold the velocities for boat mode
	global BoatVx
	global BoatVy
	global BoatWz
	#Variables that hold the input Velocities/Forces - depending on the mode
	global Vx
	global Vy
	global Wz
	
	
	pause = twist.angular.x
	if pause == 1:
		BoatVx = 0
		BoatVy = 0
		BoatWz = 0
	else:
		Vx = twist.linear.x*2
		Vy = twist.linear.y*2
		Wz = twist.angular.z/2
	

	
# Radius of the wheel in m
global R
R=0.05
#Distance from wheel center to x axis from body fram in m
global lx
lx=0.15
#Distance from wheel center to y axis from body frame in m
global ly
ly=0.15
global Vx
global Vy
global Wz
Vx=0
Vy=0
Wz=0
#Mass in kg
global M
M=20
#moment of inertia in kgm^2
global I
I=0.61
#Non linear drag coeff in x
global Cu
Cu=-29
#Non linear drag coeff in y
global Cv
Cv=-25
#Non linear drag coeff for rotation
global Cr
Cr=-1
#linear drag coeff in x
global Cu_l
Cu_l=2
#linear drag coeff in y
global Cv_l
Cv_l=2
#linear drag coeff for rotation
global Cr_l
Cr_l=0.1			
	
#Create a node and subscribe it to the messege send from the joystick
rospy.init_node('marin_node')
sub = rospy.Subscriber('mallard/cmd_vel', Twist, callback)


global pub
pub=rospy.Publisher('mallard/cmd_wheel', Twist, queue_size=1)

#Wheel ot Twist message type. 
global Wheel
Wheel=Twist()


#Initialise mode and wheel_velocity. Needed in order to use the variables in the callback
wheel_velocity=[0,0,0,0]

#Value used to calculate the angle of rotation. Chosen small to approximate dt-from a derivative
deltaT=0.01

rate=rospy.Rate(100) #100Hz loop in order to have deltaT=10ms

BoatWz=0
BoatVx=0
BoatVy=0


while not rospy.is_shutdown():
	#Allocate velocities for mode 2 - Boat mode. Here Vx,Vy represent forces and Wz is torque
	#This mode is done in the loop, because the velocities depend not only on current input,but
	#also on the previous ones
	rate.sleep()

		#Calculate the yaw angle using the angular velocity of the boat
	Angle=BoatWz*deltaT

		#Go from body frame velocities to inertial frame
	dummy = BoatVx
	BoatVx=BoatVx*math.cos(Angle)+BoatVy*math.sin(Angle)
	BoatVy=BoatVy*math.cos(Angle)-dummy*math.sin(Angle)
			
		#Calculate next velocities by using current ones, input forces and drag
	BoatVx=BoatVx+(((Cu*BoatVx*abs(BoatVx)-Cu_l*BoatVx+Vx)*deltaT)/M)
	BoatVy=BoatVy+(((Cv*BoatVy*abs(BoatVy)-Cv_l*BoatVy+Vy)*deltaT)/M)
	BoatWz=BoatWz+(((Cr*BoatWz*abs(BoatWz)-Cr_l*BoatWz+Wz)*deltaT)/I)
			
	
	#Find the velocity of each wheel
	wheel_velocity=transform_velocities(BoatVx,BoatVy,BoatWz)	
		#Assign them to a variable of the message type and publish them
	Wheel.linear.x=wheel_velocity[0]
	Wheel.linear.y=wheel_velocity[1]
	Wheel.linear.z=wheel_velocity[2]
	Wheel.angular.x=wheel_velocity[3]
	Wheel.angular.y=2
	pub.publish(Wheel)
	
	rate=rospy.Rate(100)

