#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  RCLCPP_INFO(logger, "Hi!");
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  RCLCPP_INFO(logger, "found manipulator");
  
  // move_group_interface.setPoseTarget(target_pose, "link_6");
//collision object
  // auto const collision_object = [frame_id =
  //                                 move_group_interface.getPlanningFrame()] {
  //   moveit_msgs::msg::CollisionObject collision_object;
  //   collision_object.header.frame_id = frame_id;
  //   collision_object.id = "box1";
  //   shape_msgs::msg::SolidPrimitive primitive;

  //   // Define the size of the box in meters
  //   primitive.type = primitive.BOX;
  //   primitive.dimensions.resize(3);
  //   primitive.dimensions[primitive.BOX_X] = 0.3;
  //   primitive.dimensions[primitive.BOX_Y] = 0.1;
  //   primitive.dimensions[primitive.BOX_Z] = 0.3;

  //   // Define the pose of the box (relative to the frame_id)
  //   geometry_msgs::msg::Pose box_pose;
  //   box_pose.orientation.w = 1.0;
  //   box_pose.position.x = 0.3;
  //   box_pose.position.y = 0.1;
  //   box_pose.position.z = 1.2;

  //   collision_object.primitives.push_back(primitive);
  //   collision_object.primitive_poses.push_back(box_pose);
  //   collision_object.operation = collision_object.ADD;

  //   return collision_object;
  // }();

  // //add the collision object
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // planning_scene_interface.applyCollisionObject(collision_object);

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::PoseStamped msg;  
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 0.549;
      msg.pose.orientation.y = -0.446;
      msg.pose.orientation.z = 0.446;
      msg.pose.orientation.w = 0.549;
      msg.pose.position.x = -0.406;
      msg.pose.position.y = 0.256;
      msg.pose.position.z = 0.818;
      return msg;
    return msg;
  }();
  //Plan and execute a multi-step sequence using Pilz
  move_group_interface.setPoseTarget(target_pose, "ft_frame");
  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_interface.setPlannerId("PTP");
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  auto const end_pose = []{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose.orientation.x = 0.549;
    msg.pose.orientation.y = -0.446;
    msg.pose.orientation.z = 0.446;
    msg.pose.orientation.w = 0.549;
    msg.pose.position.x = -0.406;
    msg.pose.position.y = 0.256;
    msg.pose.position.z = 0.600;
    return msg;
  }();

  move_group_interface.setPoseTarget(end_pose, "ft_frame");
  //move_group_interface.setPlanningPipelineId("ompl");
  move_group_interface.setPlannerId("LIN");
  auto const [success1, plan1] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success1) {
    move_group_interface.execute(plan1);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  // spinner.join();
  return 0;
}
