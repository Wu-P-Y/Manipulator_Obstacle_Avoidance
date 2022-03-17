'''
move sawyer in cartesian space

该脚本尚未测试
'''


import rospy
import numpy as np
from intera_motion_interface import (   #on-board path planner
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
from tf_conversions import posemath
from intera_interface import Limb

import rot2q
import transformation

def move_cartesian_space(path, quaternion):
    z = 0.05    # 机械臂末端高度
    for point in path:
        point.append(z)

    try:
        rospy.init_node('move_cartesian_space') 
        limb = Limb()

        traj_options = TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

        wpt_opts = MotionWaypointOptions(max_linear_speed=0.6,
                                         max_linear_accel=0.6,
                                         max_rotational_speed=1.57,
                                         max_rotational_accel=1.57,
                                         max_joint_speed_ratio=1.0)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        joint_names = limb.joint_names()

        for point in path:
            endpoint_state = limb.tip_state('right_hand')
            if endpoint_state is None:
                    rospy.logerr('Endpoint state not found with tip name %s', 'right_hand')
                    return None
            pose = endpoint_state.pose

            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = point[2]

            poseStamped = PoseStamped()
            poseStamped.pose = pose

            joint_angles = limb.joint_ordered_angles()
            waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)

            rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

            traj.append_waypoint(waypoint.to_msg())

            result = traj.send_trajectory(timeout=None)
            if result is None:
                rospy.logerr('Trajectory FAILED to send')
                return

            if result.result:
                rospy.loginfo('Motion controller successfully finished the trajectory!')
            else:
                rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)

    except:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')