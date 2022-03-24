#!/usr/bin/env python



path = [[0.018485065043501767, 0.023628422933778485], [0.02611551830024211, -0.014764135731891628], [0.004943316748834494, -0.00869273371964725], 
		[0.0025982598472264317, 0.00427189018659081], [0.02937110954758551, 0.006110476572554027], [0.026820185935293672, 0.01344163778697644], 
		[0.005076738343857308, 0.029567325340449854], [0.015269561886653661, 0.02582325463201057], [0.007313363929998715, 0.0068201692081204255], 
		[-0.011954631130711295, 0.009060176296774181], [0.011036906550405448, 0.02789599780967908], [0.019362315665727544, 0.022915076523126104], 
		[0.01936231566572747, 0.022915076523126104], [0.019362315665727648, 0.022915076523126104]]

import rospy
import intera_interface
from intera_interface import CHECK_VERSION

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion
	)

rospy.init_node('fk_ik')
rs = intera_interface.RobotEnable(CHECK_VERSION)
limb = intera_interface.Limb('right')

rs.enable
limb.set_joint_position_speed(0.05)

print (limb.endpoint_pose())

for p in path:
	current_pose = limb.endpoint_pose()
	go_to_pose = Pose()
	go_to_pose.position.x = current_pose['position'].x + p[0]
	go_to_pose.position.y = current_pose['position'].y + p[1]
	go_to_pose.position.z = current_pose['position'].z
	go_to_pose.orientation = current_pose['orientation']

	joint_angles = limb.ik_request(go_to_pose, 'right_hand')
	limb.move_to_joint_positions(joint_angles)

	print (limb.endpoint_pose())