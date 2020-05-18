#! /usr/bin/python

import PyKDL
import sys
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from lab_4.srv import jintServ
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import os
import json
from PyKDL import *
from collections import OrderedDict


path = Path()
freq = 50

def getParams():
	file_path = os.path.realpath(__file__)
	with open(os.path.dirname(file_path) + '/../convert/dh.json') as input_file:
		params = json.loads(input_file.read(), object_pairs_hook=OrderedDict)
	return params

def is_allowed_interpolation_request(req):
	if not 0 <= req.joint1 <= 0.2:
			rospy.logerr("Blad. Wartosc joint1 powinna byc w zakresie 0 do 0.2.")
			return False
	if not -0.2 <= req.joint2 <= 0:
			rospy.logerr("Blad. Wartosc joint2 powinna byc w zakresie -0.2 do 0.")
			return False
	if not -0.2 <= req.joint3 <= 0:
			rospy.logerr("Blad. Wartosc joint3 powinna byc w zakresie -0.2 do 0.")
			return False

	if req.time <= 0:
		rospy.logerr("Blad. Czas powinien byc wiekszy od 0.")
		return False

	if req.int_type != "quadratic" and req.int_type != "linear":
		rospy.logerr("Blad. Niepoprawny typ interpolacji. Poprawne typy: linear i quadratic.")
		return False

	return True


def linear_int(start_position, end_position, time, current_time):
	return start_position + (float(end_position - start_position) / time) * current_time


def quadratic_int(start_position, end_position, time, current_time, a):
	if current_time < time / 2.:
		return start_position + a * current_position**2
	else:
		return end_position - a * (time - current_time)**2

def create_empty_joint_state():
	empty_joint_state = JointState()
	empty_joint_state.header = Header()
	empty_joint_state.header.stamp = rospy.Time.now()
	empty_joint_state.name = ['base_to_link_1', 'link1_to_link_2', 'link2_to_link_3']
	empty_joint_state.velocity = []
	empty_joint_state.effort = []
	return empty_joint_state

def compute_coefs(start_pos, request_pos, time): # wspolczynniki do interpolacji f kwadratowa
	a = []
	for i in range(0, len(start_pos)):
		a.append(2. * float(request_pos[i] - start_pos[i]) / time**2)
	return a

def publish_path(data):
    chain = PyKDL.Chain()
    frame = PyKDL.Frame()
    k=1
    prev_d=0
    prev_th=0
    n_joints = len(params.keys())
    for i in params.keys():
        a, d, alpha, th = params[i]
        alpha, a, d, th = float(alpha), float(a), float(d), float(th)
        joint = PyKDL.Joint(PyKDL.Joint.TransZ)
	if k!=1:
            fr = frame.DH(a, alpha, prev_d, prev_th)
            segment = PyKDL.Segment(joint, fr)
            chain.addSegment(segment)
	k=k+1
	prev_d=d
	prev_th=th

    a, d, alpha, th = params["i3"]
    chain.addSegment(PyKDL.Segment(joint,frame.DH(0,0,d,th)))

    joints = PyKDL.JntArray(n_joints)
    for i in range(n_joints):
	joints[i] = int(data.position[i])


    fk=PyKDL.ChainFkSolverPos_recursive(chain)
    finalFrame=PyKDL.Frame()
    fk.JntToCart(joints,finalFrame)
    quaterions = finalFrame.M.GetQuaternion()

    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = data.header.stamp
    pose.pose.position.x = finalFrame.p[0]
    pose.pose.position.y = finalFrame.p[1] 
    pose.pose.position.z = finalFrame.p[2]
    pose.pose.orientation.x = quaterions[0]
    pose.pose.orientation.y = quaterions[1]
    pose.pose.orientation.z = quaterions[2]
    pose.pose.orientation.w = quaterions[3]
    path.header = pose.header
    path.poses.append(pose)
    path_pub.publish(path)


def handle_interpolation_request(req):

	if not is_allowed_interpolation_request(req):
		return False

	start_pos = rospy.wait_for_message('joint_states', JointState, timeout = 10).position
	request_pos = [req.joint1, req.joint2, req.joint3]

	rate = rospy.Rate(freq)
	frames_number = int(math.ceil(req.time * freq))
	current_time = 0.

	if req.int_type == "linear":
		for k in range(0, frames_number + 1):
			joint_state = create_empty_joint_state()
			expected_positions = []
			for i in range(0, 3):
				expected_positions.append(linear_int(start_pos[i], request_pos[i], req.time, current_time))

			joint_state.position = expected_positions
			pub.publish(joint_state)
			publish_path(joint_state)
			current_time = current_time + 1. / freq
			rate.sleep()
	else:
		a = compute_coefs(start_pos, request_pos, req.time) # wspolczynnki funkcji kwadratowych przy drugiej potedze
		for k in range(0, frames_number + 1):
			joint_state = create_empty_joint_state()
			positions = []
			for i in range(0, 3):
				positions.append(quadratic_int(start_pos[i], request_pos[i], req.time, current_time, a[i]))

			joint_state.position = positions
			pub.publish(joint_state)
			publish_path(joint_state)
			current_time = current_time + 1. / freq
			rate.sleep()

	return True


if __name__ == "__main__":
	params = getParams()
	rospy.init_node('jint')
	pub = rospy.Publisher('interpolation', JointState, queue_size = 10)
	path_pub = rospy.Publisher('jint_path', Path, queue_size=10)
	s = rospy.Service('jint_control_srv', jintServ, handle_interpolation_request)
	rospy.spin()
