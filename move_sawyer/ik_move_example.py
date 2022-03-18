'''
参考https://www.youtube.com/watch?v=vpi19k0Bu_I

使用自带的逆运动学求解方法求解笛卡尔坐标对应的关节坐标并移动机器人
'''


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
limb.set_joint_position_speed(0.1)
limb.move_to_neutral()

current_pose = limb.endpoint_pose()
go_to_pose = Pose()
go_to_pose.position.x = current_pose['position'].x
go_to_pose.position.y = current_pose['position'].y + 1000	# 1000mm
go_to_pose.position.z = current_pose['position'].z
go_to_pose.orientation = current_pose['orientation']

joint_angles = limb.ik_request(go_to_pose, 'right_hand')
limb.move_to_joint_positions(joint_angles)

print limb.endpoint_pose()