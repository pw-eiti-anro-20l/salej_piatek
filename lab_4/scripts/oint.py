#!/usr/bin/env python

import rospy
from lab_4.srv import OintServiceStruct
from geometry_msgs.msg import PoseStamped
import math
from nav_msgs.msg import Path


path = Path()
freq = 50
prev_pos = [0., 0., 0., 0., 0., 0., 1.]


def is_interpolation_allowed(req):
	if req.time <= 0.:
		print "Blad. Czas musi byc wiekszy od zera."
		return False
	if req.int_type != "linear" and req.int_type != "quadratic":
		print "Blad. Niedozwolony typ interpolacji. Dozwolone typy: linear i quadratic."
		return False
	return True

def fill_poseStamped(position):
	p = PoseStamped()
	p.header.stamp = rospy.Time.now()
	p.header.frame_id = 'base_link'
	p.pose.position.x = position[0]
	p.pose.position.y = position[1]
	p.pose.position.z = position[2]
	p.pose.orientation.x = position[3]
	p.pose.orientation.y = position[4]
	p.pose.orientation.z = position[5]
	p.pose.orientation.w = position[6]
	return p


def linear_int(start_p, end_p, time, current_time):
	return start_p + (float(end_p - start_p) / time) * current_time



def quadratic_int(start_p, end_p, time, current_time, a):
	if current_time < time / 2:
		return start_p + a * current_time**2
	else:
		return end_p - a * (time - current_time)**2


def compute_coefs(start_pos, request_pos, time): # wspolczynniki do interpolacji f kwadratowa
	a = []
	for i in range(0, len(start_pos)):
		a.append(2. * float(request_pos[i] - start_pos[i]) / time**2)
	return a

def publish_path(pose):
	path.header = pose.header
	path.poses.append(pose)
	path_pub.publish(path)


def handle_interpolation(req):
	
	if not is_interpolation_allowed(req):
		return False

	global prev_pos

	new_pos = [req.x, req.y, req.z, req.qx, req.qy, req.qz, req.qw]

	rate = rospy.Rate(freq)

	current_time = 0.
	frames_number = int(math.ceil(req.time * freq))

	position = [None] * 7
	if req.int_type == "linear":
		
		for i in range(frames_number + 1):
			for k in range(0, 7):
				position[k] = linear_int(prev_pos[k], new_pos[k], req.time, current_time)

			pose = fill_poseStamped(position)
			pub.publish(pose)
			publish_path(pose)
			current_time = current_time + 1. / freq
			rate.sleep()

	else:
		a = compute_coefs(prev_pos, new_pos, req.time)
		for i in range(frames_number + 1):
			for k in range(0, 7):
				position[k] = quadratic_int(prev_pos[k], new_pos[k], req.time, current_time, a[k])

			pose = fill_poseStamped(position)
			pub.publish(pose)
			publish_path(pose)
			current_time = current_time + 1. / freq
			rate.sleep()

	prev_pos = new_pos
	return True


if __name__ == "__main__":
	rospy.init_node('oint')
	pub = rospy.Publisher('/oint_pose', PoseStamped, queue_size=10)
	path_pub = rospy.Publisher('oint_path', Path, queue_size=10)
	service = rospy.Service('oint_control_srv', OintServiceStruct, handle_interpolation)
	rospy.spin()
