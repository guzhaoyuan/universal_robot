// moveit ros api for adding collision

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_ros_api");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;

  // Visualization
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Advertise the required topic

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "base_link";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "base_link";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.4;
  pose.position.y = 0.0;
  pose.position.z = 0.15;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.2;
  primitive.dimensions[1] = 0.2;
  primitive.dimensions[2] = 0.3;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  attached_object.object.operation = attached_object.object.ADD;

  ROS_INFO("Adding the object into the world at the location of the hand.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualTioolsGui window to continue the demo");

  ros::shutdown();
  return 0;
}
