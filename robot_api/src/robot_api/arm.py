#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import actionlib
import rospy

from .arm_joints import ArmJoints

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal

from trajectory_msgs.msg import JointTrajectoryPoint

ACTION_NAME = "arm_controller/follow_joint_trajectory"


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # TODO: Create actionlib client
        self._client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
        # TODO: Wait for server
        self._client.wait_for_server()
        pass

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        point = JointTrajectoryPoint()
        # TODO: Set position of trajectory point
        point.positions.extend(arm_joints.values())
        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Duration(secs=5)

        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()
        # TODO: Add joint name to list
        goal.trajectory.joint_names.extend(ArmJoints.names())
        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points.append(point)

        # TODO: Send goal
        self._client.send_goal(goal)
        # TODO: Wait for result
        self._client.wait_for_result()
