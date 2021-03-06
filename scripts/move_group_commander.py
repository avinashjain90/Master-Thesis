#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_ros_planning_interface import _moveit_planning_scene_interface
from geometry_msgs.msg import PoseStamped, Pose, Point
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
#from exception import MoveItCommanderException
from moveit_ros_planning_interface._moveit_move_group_interface import *
import tf
group = MoveGroup("manipulator", "robot_description")

from std_msgs.msg import String
object_pose = geometry_msgs.msg.Pose()

def target_pose(data):
  object_pose = data

def move_group_python_interface_tutorial():
  
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_commander',anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("manipulator")


  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  #print "============ Waiting for RVIZ..."
  rospy.sleep(10)
  #print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  #group.set_planning_frame() = 'base_link'
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"

  #set_io = on
  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
    # print "============ Generating plan 1"
    #listener = tf.TransformListener()

    # #  while not rospy.is_shutdown():
    # try:
    #     listener.waitForTransform('object_frame', 'base_link',rospy.Time.now(), rospy.Duration(4.0))
    #     (trans,rot) = listener.lookupTransform('object_frame', 'base_link', rospy.Time.now())
    #     print "trans"

    #     print trans

    #     print "rot"

    #     print rot
    # except (tf.LookupException, tf.ConnectivityException):
    #   print "123"
  pose_target = geometry_msgs.msg.Pose()
    
  pose_target.position.x = 0.01
  pose_target.position.y = 0.23
  pose_target.position.z = 0.69
  pose_target.orientation.x = 0.6
  pose_target.orientation.y = 0.76
  pose_target.orientation.z = -0.43
  pose_target.orientation.w = 0.46
    # #Eigen::Affine3d pose = Eigen::Translation3d(0.028, 0.793, 0.390)
    #                       # * Eigen::Quaterniond(-0.014, 0.733, 0.680, -0.010);
    # #group.setPoseTarget(pose);
  group.set_pose_target(pose_target)
  #group.set_pose_target(object_pose)
  #group.set_named_target("up")
  #plan1 = group.get_random_joint_values()

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  group.set_planner_id("RRTConnectkConfigDefault")
  group.set_planning_time(50)
  plan1 = group.plan()
  group.execute(plan1)

  #print "============ Waiting while RVIZ displays plan1..."
  #rospy.sleep(5)

 

    # display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    # display_trajectory.trajectory_start = robot.get_current_state()
    # display_trajectory.trajectory.append(plan1)
    # display_trajectory_publisher.publish(display_trajectory);

  #print "============ Waiting while plan1 is visualized (again)..."
  #rospy.sleep(5)

  # Uncomment below line when working with a real robot
  #group.go(wait=True)
  
  # Use execute instead if you would like the robot to follow 
  # the plan that has already been computed
  #group.execute(plan1)
  #set_io = on
  #group.clear_pose_targets()

  ## Then, we will get the current set of joint values for the group
  #joint_angles = group.get_current_joint_values()
  #print "============ Joint values: ", joint_angles

  ## Now, let's modify one of the joints, plan to the new joint
  ## space goal and visualize the plan
  #joint_angles = [0.0,0.0,0.0,0.0,0.0,0.0]
  #joint_angles[0] = 1.0
  #group.set_joint_value_target(joint_angles)

  #plan2 = group.plan()
  #display_trajectory.trajectory.append(plan2)
  #display_trajectory_publisher.publish(display_trajectory);
  #group.execute(plan2)
  joint_angles = group.get_current_joint_values()
  #print "============ Waiting while RVIZ displays plan2..."
  #rospy.sleep(5)
 ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
  print "============ STOPPING"

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
    #rospy.Subscriber("target_pose", Pose(), target_pose)
  except rospy.ROSInterruptException:
    pass
