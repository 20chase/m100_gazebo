import rospy
import time
import math

import dji_sdk.msg

import numpy as np

from geometry_msgs.msg import Twist

# hyper parameters

KP_X = 5
KP_Y = 5
KP_Z = 5

KI_X = 0.01
KI_Y = 0.01

KD_X = 1
KD_Y = 1
KD_Z = 1

global position
global velocity
global acceleration

position = np.zeros(3)
velocity = np.zeros(3)
acceleration = np.zeros(3)

def position_callback(data):
	global position

	position[0] = data.x
	position[1] = -data.y
	position[2] = data.z

def velocity_callback(data):
	global velocity

	velocity[0] = data.vx
	velocity[1] = -data.vy
	velocity[2] = data.vz

def acceleration_callback(data):
	global acceleration

	acceleration[0] = data.ax
	acceleration[1] = data.ay
	acceleration[2] = data.az


def creat_trajectory(center_pos, radius):
	target_pos = np.zeros([8000, 3])
	phi = 0
	for i in xrange(8000):
		target_pos[i][0] = center_pos[0] + radius * math.cos(phi / 50)
		target_pos[i][1] = center_pos[1] + radius * math.sin(phi / 50) 
		target_pos[i][2] = 2
		phi += 1
		radius += 0.001

	return target_pos


if True:
	rospy.init_node('control_m100')
	control_rate = rospy.Rate(33)

	position_pub = rospy.Publisher('/drl/control_pos', Twist, queue_size = 1)
	velocity_pub = rospy.Publisher('/drl/control_vel', Twist, queue_size = 1)
	attitude_pub = rospy.Publisher('/drl/control_att', Twist, queue_size = 1)

	rospy.Subscriber('/dji_sdk/local_position', dji_sdk.msg.LocalPosition, position_callback)
	rospy.Subscriber('/dji_sdk/velocity', dji_sdk.msg.Velocity, velocity_callback)
	rospy.Subscriber('/dji_sdk/acceleration', dji_sdk.msg.Acceleration, acceleration_callback)

	time.sleep(1)

	pos_msg = Twist()
	vel_msg = Twist()
	att_msg = Twist()

	origin_position = np.zeros(3)

	origin_position[0] = position[0]
	origin_position[1] = position[1]
	origin_position[2] = position[2]

	relative_position = np.zeros(3)

	trajectory = creat_trajectory(position, 1)

	trajectory_num = 0

	V = 2
	R = 2
	H = 2

	exp_num = 0

	while not rospy.is_shutdown():

		d_x = input('enter x position:')
		d_y = input('enter y position:')
		d_z = input('enter z position:')

		end_flag = 0

		file_name = '/home/uav/exp_data/fly_point_{}.csv'.format(exp_num)

		file = open(file_name, 'w')

		err_x = 0
		err_y = 0

		while True:

			

			relative_position[0] = position[0] - origin_position[0]
			relative_position[1] = position[1] - origin_position[1]
			relative_position[2] = position[2] - origin_position[2]



			control_x = KP_X * (d_x - velocity[0]) + KI_X * (err_x) + KD_X * (0 - acceleration[0])
			control_y = KP_Y * (d_y - velocity[1]) + KI_Y * (err_y) + KD_Y * (0 - acceleration[1])
			control_z = d_z 

			err_x += (d_x - velocity[0])
			err_y += (d_y - velocity[1])

			# control_x = KP_X * (d_x - relative_position[0]) + KD_X * (0 - velocity[0])
			# control_y = KP_Y * (d_y - relative_position[1]) + KD_Y * (0 - velocity[1])
			# control_z = KP_Z * (d_z - relative_position[2]) + KD_Z * (0 - velocity[2])

			file.write('{}, {}, {}, {}, {}, {} \n'.format(d_x, velocity[0], d_y, -velocity[1], d_z, velocity[2]))

			print 'control_x: ', control_x
			print 'control_y: ', control_y
			print 'control_z: ', control_z

			att_msg.linear.x = 0
			att_msg.linear.y = 0
			att_msg.linear.z = control_z
			att_msg.angular.x = -control_y
			att_msg.angular.y = -control_x
			att_msg.angular.z = 0

			attitude_pub.publish(att_msg)

			if (abs(control_x) < 0.1) and (abs(control_y) < 0.1) and (abs(control_z) < 0.1):
				end_flag += 1
				if end_flag > 15:
					break

			else:
				end_flag = 0



			control_rate.sleep()

		file.close()

		exp_num += 1

		# while True:

			

		# 	vx = V * math.sin((V/R)*trajectory_num/50.0)
		# 	vy = V * math.cos((V/R)*trajectory_num/50.0)

		# 	file.write('{}, {}, {} \n'.format(position[0], position[1], position[2]))

		# 	R += 0.001
		# 	H += 0.001
			
		# 	pos_msg.linear.x = vx
		# 	pos_msg.linear.y = vy
		# 	pos_msg.linear.z = H

		# 	pos_msg.angular.x = 0
		# 	pos_msg.angular.y = 0
		# 	pos_msg.angular.z = 0

		# 	position_pub.publish(pos_msg)

		# 	trajectory_num += 1

		# 	time.sleep(0.01)







