#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

static const rclcpp::Logger LOGGER =
  rclcpp::get_logger("ur10e_robot1_moveit");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ur10e_robot1_moveit");

  // Executor required by MoveIt
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // Wait for /clock if using sim time
  while (rclcpp::ok() && !node->get_clock()->now().nanoseconds()) {
    RCLCPP_INFO(LOGGER, "Waiting for simulated time...");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  // 🔑 THIS MUST MATCH YOUR MoveIt SRDF GROUP
  const std::string ARM_GROUP = "robot1";

  moveit::planning_interface::MoveGroupInterface move_group(node, ARM_GROUP);

  move_group.setPoseReferenceFrame("world");
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  RCLCPP_INFO(LOGGER, "Planning frame: %s",
              move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(LOGGER, "End effector link: %s",
              move_group.getEndEffectorLink().c_str());

  // Simple reachable pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.5;

  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  auto result = move_group.plan(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Planning failed!");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(LOGGER, "Planning successful, executing...");
  move_group.execute(plan);

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(LOGGER, "Motion complete.");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
