import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL


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


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Current robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_init_state(self):
    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -2.2339676062213343
    joint_goal[1] = -1.8633605442442835
    joint_goal[2] = -2.18131160736084
    joint_goal[3] = -0.7273023885539551
    joint_goal[4] = 1.5918922424316406
    joint_goal[5] = 0.11884832382202148

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_goal_state(self):
    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -0.5023930708514612
    joint_goal[1] = -1.9044658146300257
    joint_goal[2] = -2.186142921447754
    joint_goal[3] = -0.5610111516765137
    joint_goal[4] = 1.6421265602111816
    joint_goal[5] = 0.12065744400024414

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def plan_to_init(self):
    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -2.2339676062213343
    joint_goal[1] = -1.8633605442442835
    joint_goal[2] = -2.18131160736084
    joint_goal[3] = -0.7273023885539551
    joint_goal[4] = 1.5918922424316406
    joint_goal[5] = 0.11884832382202148

    plan = move_group.plan(joint_goal)
    return plan


  def plan_to_goal(self):
    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -0.5023930708514612
    joint_goal[1] = -1.9044658146300257
    joint_goal[2] = -2.186142921447754
    joint_goal[3] = -0.5610111516765137
    joint_goal[4] = 1.6421265602111816
    joint_goal[5] = 0.12065744400024414

    plan = move_group.plan(joint_goal)
    return plan


  def execute_plan(self, plan):
    move_group = self.move_group

    move_group.execute(plan, wait=True)



  def wait_for_state_update(self, name, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


  def add_box(self, timeout=2):
    scene = self.scene
    
    box_height = 0.3

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.y = 0.4
    box_pose.pose.position.z = box_height/2
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.28, 0.28, box_height))

    return self.wait_for_state_update(name=box_name, box_is_known=True, timeout=timeout)
  

  def add_table(self, timeout=2):
    scene = self.scene

    table_height = 0.5
    
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.y = 0.15
    box_pose.pose.position.z = -table_height/2
    box_name = "table"
    scene.add_box(box_name, box_pose, size=(0.45, 0.45, table_height))

    return self.wait_for_state_update(name=box_name, box_is_known=True, timeout=timeout)


  def add_wall(self, timeout=4):
    scene = self.scene

    wall_pose = geometry_msgs.msg.PoseStamped()
    wall_pose.header.frame_id = "base_link"
    wall_pose.pose.orientation.w = 1.0
    wall_pose.pose.position.y = -0.33
    wall_name = "wall_back"
    scene.add_box(wall_name, wall_pose, size=(1., 0.001, 1.5))

    wall_pose.pose.orientation.w = 0.707
    wall_pose.pose.orientation.x = 0.707
    wall_pose.pose.position.y = 0.
    wall_pose.pose.position.z = 1.
    wall_name = "wall_top"
    scene.add_box(wall_name, wall_pose, size=(1., 0.001, 1.5))

    wall_pose.pose.orientation.w = 0.707
    wall_pose.pose.orientation.x = 0.0
    wall_pose.pose.orientation.z = 0.707
    wall_pose.pose.position.z = 0.0
    wall_pose.pose.position.x = 0.45
    wall_name = "wall_right"
    scene.add_box(wall_name, wall_pose, size=(1., 0.001, 1.5))

    wall_pose.pose.position.x = -0.45
    wall_name = "wall_left"
    scene.add_box(wall_name, wall_pose, size=(1., 0.001, 1.5))

    return self.wait_for_state_update(name=wall_name, box_is_known=True, timeout=timeout)

  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:

    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to init planning scene ..."
    raw_input()
    tutorial.add_box()
    tutorial.add_table()
    tutorial.add_wall()

    print "============ Press `Enter` to plan to init state ..."
    raw_input()
    plan = tutorial.plan_to_init()
    tutorial.display_trajectory(plan)

    print "============ Press `Enter` to execute ..."
    raw_input()
    tutorial.execute_plan(plan)

    print "============ Press `Enter` to plan to goal state ..."
    raw_input()
    plan = tutorial.plan_to_goal()
    tutorial.display_trajectory(plan)

    print "============ Press `Enter` to execute ..."
    raw_input()
    tutorial.execute_plan(plan)

    print "============ Demo complete!"

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()