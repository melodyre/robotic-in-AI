#!/usr/bin/env python3
# File name   : move.py
# Description : Control Motor
# Product     : GWR
# Website     : www.gewbot.com
# Author      : William
# Date        : 2019/07/24
import time
import numpy as np
import math
import RPi.GPIO as GPIO

# motor_EN_A: Pin7  |  motor_EN_B: Pin11
# motor_A:  Pin8,Pin10    |  motor_B: Pin13,Pin12

Motor_A_EN    = 4
Motor_B_EN    = 17

Motor_A_Pin1  = 26
Motor_A_Pin2  = 21
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

Dir_forward   = 0
Dir_backward  = 1

left_forward  = 1
left_backward = 0

right_forward = 0
right_backward= 1

pwn_A = 0
pwm_B = 0

def motorStop():#Motor stops
	GPIO.output(Motor_A_Pin1, GPIO.LOW)
	GPIO.output(Motor_A_Pin2, GPIO.LOW)
	GPIO.output(Motor_B_Pin1, GPIO.LOW)
	GPIO.output(Motor_B_Pin2, GPIO.LOW)
	GPIO.output(Motor_A_EN, GPIO.LOW)
	GPIO.output(Motor_B_EN, GPIO.LOW)


def setup():#Motor initialization
	global pwm_A, pwm_B
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(Motor_A_EN, GPIO.OUT)
	GPIO.setup(Motor_B_EN, GPIO.OUT)
	GPIO.setup(Motor_A_Pin1, GPIO.OUT)
	GPIO.setup(Motor_A_Pin2, GPIO.OUT)
	GPIO.setup(Motor_B_Pin1, GPIO.OUT)
	GPIO.setup(Motor_B_Pin2, GPIO.OUT)

	motorStop()
	try:
		pwm_A = GPIO.PWM(Motor_A_EN, 1000)
		pwm_B = GPIO.PWM(Motor_B_EN, 1000)
	except:
		pass


def motor_left(status, direction, speed):#Motor 2 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_B_Pin1, GPIO.LOW)
		GPIO.output(Motor_B_Pin2, GPIO.LOW)
		GPIO.output(Motor_B_EN, GPIO.LOW)
	else:
		if direction == Dir_backward:
			GPIO.output(Motor_B_Pin1, GPIO.HIGH)
			GPIO.output(Motor_B_Pin2, GPIO.LOW)
			pwm_B.start(100)
			pwm_B.ChangeDutyCycle(speed)
		elif direction == Dir_forward:
			GPIO.output(Motor_B_Pin1, GPIO.LOW)
			GPIO.output(Motor_B_Pin2, GPIO.HIGH)
			pwm_B.start(0)
			pwm_B.ChangeDutyCycle(speed)


def motor_right(status, direction, speed):#Motor 1 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_A_Pin1, GPIO.LOW)
		GPIO.output(Motor_A_Pin2, GPIO.LOW)
		GPIO.output(Motor_A_EN, GPIO.LOW)
	else:
		if direction == Dir_forward:#
			GPIO.output(Motor_A_Pin1, GPIO.HIGH)
			GPIO.output(Motor_A_Pin2, GPIO.LOW)
			pwm_A.start(100)
			pwm_A.ChangeDutyCycle(speed)
		elif direction == Dir_backward:
			GPIO.output(Motor_A_Pin1, GPIO.LOW)
			GPIO.output(Motor_A_Pin2, GPIO.HIGH)
			pwm_A.start(0)
			pwm_A.ChangeDutyCycle(speed)
	return direction


def move(speed, direction, turn, radius=0.1):   # 0 < radius <= 1  
	#speed = 100
	if direction == 'forward':
		if turn == 'right':
			motor_left(0, left_backward, int(speed*radius))
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(0, right_backward, int(speed*radius))
		else:
			motor_left(1, left_forward, speed)
			motor_right(1, right_forward, speed)
	elif direction == 'backward':
		if turn == 'right':
			motor_left(0, left_forward, int(speed*radius))
			motor_right(1, right_backward, speed)
		elif turn == 'left':
			motor_left(1, left_backward, speed)
			motor_right(0, right_forward, int(speed*radius))
		else:
			motor_left(1, left_backward, speed)
			motor_right(1, right_backward, speed)
	elif direction == 'no':
		if turn == 'right':
			motor_left(1, left_backward, speed)
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(1, right_backward, speed)
		else:
			motorStop()
	else:
		pass




def destroy():
	motorStop()
	GPIO.cleanup()             # Release resourceS

def fo_trace(rx,ry):

	cur_co=np.array([rx[0],ry[0]])   # set up initial  coordinate

	delta_s=[]    # comperative direction, a list of the moving vector
	for i in range(len(rx)):
		delta_s.append(np.array([rx[i],ry[i]])-cur_co )
		cur_co=np.array([rx[i],ry[i]])

	dot_product=[]
	angle=[]
	for i in range(1, len(delta_s)-1):
		dot= np.vdot(delta_s[i]/np.linalg.norm(delta_s[i]), delta_s[i+1]/np.linalg.norm(delta_s[i+1]))
		dot_product.append(dot)
		ang=math.acos(dot)/math.pi
		if dot<0:
			angle.append(-ang)
		else:
			angle.append(ang)

	for i in range(len(angle)):
		if 0<angle[i]<0.0001:
			angle[i]=0

	return angle, delta_s




if __name__ == '__main__':
	rx=[30.0, 28.0, 28.0, 28.0, 26.0, 26.0, 24.0, 24.0, 22.0, 22.0, 22.0, 22.0, 22.0, 22.0, 20.0, 18.0, 16.0, 14.0, 12.0, 10.0]
	rx.reverse()
	ry=[30.0, 28.0, 26.0, 24.0, 22.0, 20.0, 18.0, 16.0, 14.0, 12.0, 10.0, 8.0, 6.0, 4.0, 2.0, 4.0, 6.0, 8.0, 10.0, 10.0]
	ry.reverse()
	#starts from (10,10), ends at(30,30)

	[angle, delta_s]=fo_trace(rx,ry)




	# cur_co=np.array([rx[0],ry[0]])   # current coordinate

	# delta_s=[]    # comperative direction, a list of the moving vector
	# for i in range(len(rx)):
	# 	delta_s.append(np.array([rx[i],ry[i]])-cur_co )
	# 	cur_co=np.array([rx[i],ry[i]])

	# dot_product=[]
	# angle=[]
	# for i in range(1, len(delta_s)-1):
	# 	dot= np.vdot(delta_s[i]/np.linalg.norm(delta_s[i]), delta_s[i+1]/np.linalg.norm(delta_s[i+1]))
	# 	dot_product.append(dot)
	# 	ang=math.acos(dot)/math.pi
	# 	if dot<0:
	# 		angle.append(-ang)
	# 	else:
	# 		angle.append(ang)

	# for i in range(len(angle)):
	# 	if 0<angle[i]<0.0001:
	# 		angle[i]=0



	try:
		speed_set = 100     # rad/s
		r=0.1
		setup()

		for k in range(len(angle)):
			dist=np.linalg.norm(delta_s[k+1])
			t=dist/speed_set/r
			if angle[k] >0:   # turn right
				move(speed_set, 'backward', 'left')
				time.sleep(t)
			elif angle[k] ==0:
				move(speed_set, 'backward', 'no')
				time.sleep(t)
			else:
				move(speed_set, 'backward', 'right')
				time.sleep(t)

		move(speed_set, 'backward', 'no')
		time.sleep(1)

		# make a U turn
		move(speed_set, 'backward', 'left')
		time.sleep(1.4)   # I tested with my robot, and found it takes 1.4s to turn around. 
		
		# preparing the back path
		rx.reverse()   
		ry.reverse()    
		[angle, delta_s]=fo_trace(rx,ry)

		for k in range(len(angle)):
			dist=np.linalg.norm(delta_s[k+1])
			t=dist/speed_set/r
			if angle[k] >0:   # turn right
				move(speed_set, 'backward', 'left')
				time.sleep(t)
			elif angle[k] ==0:
				move(speed_set, 'backward', 'no')
				time.sleep(t)
			else:
				move(speed_set, 'backward', 'right')
				time.sleep(t)

		move(speed_set, 'backward', 'no')
		time.sleep(1)

	
		motorStop()   
		destroy()
	except KeyboardInterrupt:
		destroy()


