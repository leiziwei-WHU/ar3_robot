#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped,Pose
import math
from math import pi
import numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from pyquaternion import Quaternion

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

def eulerAngles2rotationMat(theta, format='degree'):
  # RPY角，是ZYX欧拉角，依次绕定轴XYZ转动[rx, ry, rz]
  if format is 'degree':
      theta = [i * math.pi / 180.0 for i in theta]

  R_x = np.array([[1, 0, 0],
                  [0, math.cos(theta[0]), -math.sin(theta[0])],
                  [0, math.sin(theta[0]), math.cos(theta[0])]
                  ])

  R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                  [0, 1, 0],
                  [-math.sin(theta[1]), 0, math.cos(theta[1])]
                  ])

  R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                  [math.sin(theta[2]), math.cos(theta[2]), 0],
                  [0, 0, 1]
                  ])
  R = np.dot(R_z, np.dot(R_y, R_x))
  return R

# 旋转矩阵转换为四元数
def rotateToQuaternion(rotateMatrix):
  q = Quaternion(matrix=rotateMatrix)
  return q

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
    
    group = self.group

    joint_goal = group.get_current_joint_values()
    for i in range(6):
      print("joint "+str(i)+" angle is ", joint_goal[i])
    joint_goal[0] = 1.57
    joint_goal[1] = 0.78
    joint_goal[2] = 1.57
    joint_goal[3] = 0.01
    joint_goal[4] = -0.78
    joint_goal[5] = 0.001
    
    group.go(joint_goal, wait=True)
    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):
    
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the end-effector:
    euler_angle = [0,0,180]
    Rot = eulerAngles2rotationMat(euler_angle, 'degree')
    q = rotateToQuaternion(Rot)

    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1
    # pose_goal.orientation.x = 0
    # pose_goal.orientation.y = 0
    # pose_goal.orientation.z = 0
    # pose_goal.position.x = 0   
    # pose_goal.position.y = 0
    # pose_goal.position.z = 0.6

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.header.stamp = rospy.Time.now()

    pose_goal.pose.position.x = 0
    pose_goal.pose.position.y = 0
    pose_goal.pose.position.z = 0.6
    pose_goal.pose.orientation.x = 0
    pose_goal.pose.orientation.y = 0.0
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 1


    group.set_start_state_to_current_state()
    group.set_pose_target(pose_goal, self.eef_link)
    traj = group.plan()
    print "============ plan successfully"
    group.execute(traj)
    
    # ## Now, we call the planner to compute the plan and execute it.
    # plan = group.go(wait=True)
    # # Calling `stop()` ensures that there is no residual movement
    # group.stop()
    # # It is always good to clear your targets after planning with poses.
    # # Note: there is no equivalent function for clear_joint_value_targets()
    # group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()
    # tutorial.go_to_joint_state()
    print "============ Ar3 begin to move"
    tutorial.go_to_pose_goal()

    print "============ finish!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

