#! /usr/bin/python

import PyKDL
import json
import os
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

def forward_kinematics(data):
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
        min_joint, max_joint = scope["joint"+str(i+1)]
        if min_joint <= data.position[i] <= max_joint:
            joints[i] = data.position[i]
        else:
            rospy.logwarn("Wrong joint value")
            return

    fk=PyKDL.ChainFkSolverPos_recursive(chain)
    finalFrame=PyKDL.Frame()
    fk.JntToCart(joints,finalFrame)
    quaterions = finalFrame.M.GetQuaternion()

    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = finalFrame.p[0]
    pose.pose.position.y = finalFrame.p[1] 
    pose.pose.position.z = finalFrame.p[2]
    pose.pose.orientation.x = quaterions[0]
    pose.pose.orientation.y = quaterions[1]
    pose.pose.orientation.z = quaterions[2]
    pose.pose.orientation.w = quaterions[3]
    pub.publish(pose)

    marker = Marker()
    marker.header.frame_id = 'base_link'
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.pose.orientation.w = 1

    time = rospy.Duration()
    marker.lifetime = time
    marker.scale.x = 0.09
    marker.scale.y = 0.09
    marker.scale.z = 0.09
    marker.pose.position.x = finalFrame.p[0];
    marker.pose.position.y = finalFrame.p[1];
    marker.pose.position.z = finalFrame.p[2];
    marker.pose.orientation.x = quaterions[0];
    marker.pose.orientation.y = quaterions[1];
    marker.pose.orientation.z = quaterions[2];
    marker.pose.orientation.w = quaterions[3];
    marker.color.a = 0.7
    marker.color.r = 0.2
    marker.color.g = 0.8
    marker.color.b = 0.6
    marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node("KDL_DKIN", anonymous=True)
    params = {}
    scope = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../convert/dh.json', 'r') as file:
        params = json.loads(file.read())
	#temp = params["i3"]
	#params["i3"] = params["i1"]
	#params["i1"] = temp
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../models/limits.json', 'r') as file:
        scope = json.loads(file.read())

    pub = rospy.Publisher('kdl_pose', PoseStamped, queue_size=10)
    marker_pub = rospy.Publisher('kdl_visual', Marker, queue_size=100)

    rospy.Subscriber('joint_states', JointState, forward_kinematics)

    rospy.spin()
