import rospy
import time
import random

import dji_sdk.msg 
import dji_sdk.srv

import numpy as np

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates 
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

global attitude, angular_rate, position, velocity, status, obstacle_flag, reset_flag

attitude = np.zeros(4)
angular_rate = np.zeros(3)
position = np.zeros(3)
velocity = np.zeros(3)
status = np.zeros(5)

# hyper parameter

HORIZ_ATT  = 0x00
HORIZ_VEL  = 0x40
HORIZ_POS  = 0x80
VERT_VEL   = 0x00
VERT_POS   = 0x10
VERT_TRU   = 0x20
YAW_ANG    = 0x00
YAW_RATE   = 0x08
HORIZ_GND  = 0x00
HORIZ_BODY = 0x02
STABLE_OFF = 0x00
STABLE_ON  = 0x01


def attitude_callback(data):
	global attitude
	global angular_rate

	attitude[3] = data.q0
	attitude[0] = data.q1
	attitude[1] = -data.q2
	attitude[2] = -data.q3

	angular_rate[0] = data.wx
	angular_rate[1] = data.wy
	angular_rate[2] = data.wz

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

def state_callback(data):
	global status
	status[0] = data.pose[1].position.x
	status[1] = data.pose[1].position.y
	status[2] = data.pose[1].orientation.z
	status[3] = data.pose[1].orientation.w
	status[4] = data.pose[1].position.z

	print status[0]


def obstacle_callback(data):
	global obstacle_flag
	flag = data.data
	obstacle_flag = flag

def reset_callback(data):
	global reset_flag
	reset_flag = data.data

def position_control_callback(data):
	attitude_control_service(HORIZ_POS|VERT_POS|YAW_ANG|HORIZ_BODY|STABLE_ON, data.linear.x, data.linear.y, data.linear.z, data.angular.z)

def velocity_control_callback(data):
	velocity_control_service(frame = 0, vx = data.linear.x, vy = data.linear.y, vz = data.linear.z, yawRate = 0)

def attitude_control_callback(data):
	attitude_control_service(HORIZ_ATT|VERT_VEL|YAW_ANG|HORIZ_BODY|STABLE_ON, data.angular.x, data.angular.y, data.linear.z, data.angular.z)


def update_status(origin_pose):
	global attitude, angular_rate, position, velocity

	model_state.model_name = 'm100'

	model_state.pose.position.x = position[0] - origin_pose[0]
	model_state.pose.position.y = position[1] - origin_pose[1]

	height = position[2]

	if height < 0.2:
		height = 0.2

	model_state.pose.position.z = height

	model_state.pose.orientation.x = attitude[0] 
	model_state.pose.orientation.y = attitude[1] 
	model_state.pose.orientation.z = attitude[2]
	model_state.pose.orientation.w = attitude[3] 

	model_state.twist.linear.x = velocity[0]
	model_state.twist.linear.y = velocity[1]
	model_state.twist.linear.z = velocity[2]

	model_state.twist.angular.x = angular_rate[0]
	model_state.twist.angular.y = angular_rate[1]
	model_state.twist.angular.z = angular_rate[2]

	# set_pub.publish(model_state)

	set_model_state(model_state=model_state)


if __name__ == '__main__':
	rospy.init_node('dji_gazebo')
	rate = rospy.Rate(50)

	rospy.Subscriber('/dji_sdk/attitude_quaternion', dji_sdk.msg.AttitudeQuaternion, attitude_callback)
	rospy.Subscriber('/dji_sdk/local_position', dji_sdk.msg.LocalPosition, position_callback)
	rospy.Subscriber('/dji_sdk/velocity', dji_sdk.msg.Velocity, velocity_callback)
	rospy.Subscriber('/drl/control_pos', Twist, position_control_callback)
	rospy.Subscriber('/drl/control_vel', Twist, velocity_control_callback)
	rospy.Subscriber('/drl/control_att', Twist, attitude_control_callback)
	rospy.Subscriber('/gazebo/model_states', ModelStates, state_callback)
	rospy.Subscriber('/drl/check_obstacle', Int32, obstacle_callback)
	rospy.Subscriber('/drl/reset_env', Int32, reset_callback)

	terminate_pub = rospy.Publisher('/drl/terminate', Int32, queue_size=1)

	# set_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
	rospy.wait_for_service("/gazebo/set_model_state")
	set_model_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)
	model_state = ModelState()

	rospy.wait_for_service("dji_sdk/sdk_permission_control")
	sdk_permission_control_service = rospy.ServiceProxy("dji_sdk/sdk_permission_control", dji_sdk.srv.SDKPermissionControl)

	rospy.wait_for_service("dji_sdk/attitude_control")
	attitude_control_service = rospy.ServiceProxy("dji_sdk/attitude_control", dji_sdk.srv.AttitudeControl)

	rospy.wait_for_service("dji_sdk/velocity_control")
	velocity_control_service = rospy.ServiceProxy("dji_sdk/velocity_control", dji_sdk.srv.VelocityControl)

	rospy.wait_for_service("dji_sdk/drone_task_control")
	drone_task_control_service = rospy.ServiceProxy("dji_sdk/drone_task_control", dji_sdk.srv.DroneTaskControl)

	global attitude, angular_rate, position, velocity, status, obstacle_flag

	origin_pose = np.zeros(3)

	time.sleep(2)

	origin_pose[0] = position[0]
	origin_pose[1] = position[1]
	
	sdk_permission_control_service(control_enable = 1)

	time.sleep(1)
		

	rospy.loginfo("Obtained control successfully!")

	drone_task_control_service(task = 4)

	start_flag = True

	reset_flag = 0
	
	while not rospy.is_shutdown():

		if ((reset_flag == 1) or (status[0] > 42)) and (start_flag is False):

			if reset_flag == 1:
				terminate_num = 2

			if status[0] > 42:
				terminate_num = 1

			terminate_pub.publish(terminate_num)

			random_pos = random.uniform(-2,2)

			
			# modelstate = ModelState()

			# modelstate.model_name = 'm100'

			# modelstate.pose.position.x = 0
			# modelstate.pose.position.y = random_pos
			# modelstate.pose.position.z = 1.2

			# modelstate.pose.orientation.x = 0
			# modelstate.pose.orientation.y = 0
			# modelstate.pose.orientation.z = 0
			# modelstate.pose.orientation.w = 1

			# modelstate.twist.linear.x = 0
			# modelstate.twist.linear.y = 0
			# modelstate.twist.linear.z = 0

			# modelstate.twist.angular.x = 0
			# modelstate.twist.angular.y = 0
			# modelstate.twist.angular.z = 0

			# set_model_state(model_state=modelstate)


			origin_pose[0] = position[0]
			origin_pose[1] = position[1] + random_pos
			origin_pose[2] = attitude[2]


		
		else:

			if status[4] > 1:
				start_flag = False

			

			terminate_num = 0
			terminate_pub.publish(terminate_num)

		update_status(origin_pose)

		rate.sleep()


