#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <unordered_map>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <iostream>

using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleJT = rclcpp_action::ServerGoalHandle<FollowJT>;

// =====================================================
// MuJoCo globals
// =====================================================
mjModel* m = nullptr;
mjData* d = nullptr;
GLFWwindow* window = nullptr;

// Rendering
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// Joint name → qpos index
std::unordered_map<std::string, int> joint_map;

// =====================================================
// MuJoCo helpers
// =====================================================
void load_mujoco(const std::string& xml_path)
{
  char error[1000] = "";
  m = mj_loadXML(xml_path.c_str(), nullptr, error, 1000);
  if (!m) {
    throw std::runtime_error(error);
  }

  d = mj_makeData(m);

  if (!glfwInit()) {
    throw std::runtime_error("GLFW init failed");
  }

  window = glfwCreateWindow(1200, 900, "MuJoCo + MoveIt", nullptr, nullptr);
  if (!window) {
    throw std::runtime_error("GLFW window creation failed");
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // ---- MuJoCo visualization init ----
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // Nice default camera
  cam.azimuth = 90;
  cam.elevation = -20;
  cam.distance = 2.5;
}

void build_joint_map()
{
  std::vector<std::string> joints = {
    "robot1_shoulder_pan_joint",
    "robot1_shoulder_lift_joint",
    "robot1_elbow_joint",
    "robot1_wrist_1_joint",
    "robot1_wrist_2_joint",
    "robot1_wrist_3_joint"
  };

  for (const auto& name : joints)
  {
    int id = mj_name2id(m, mjOBJ_JOINT, name.c_str());
    if (id < 0) {
      throw std::runtime_error("Joint not found in MuJoCo model: " + name);
    }
    joint_map[name] = m->jnt_qposadr[id];
  }
}

// =====================================================
// ROS2 Action Server Node
// =====================================================
class MujocoTrajectoryController : public rclcpp::Node
{
public:
  MujocoTrajectoryController()
  : Node("mujoco_trajectory_controller")
  {
    action_server_ = rclcpp_action::create_server<FollowJT>(
      this,
      "/robot1_controller/follow_joint_trajectory",
      std::bind(&MujocoTrajectoryController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MujocoTrajectoryController::handle_cancel, this, std::placeholders::_1),
      std::bind(&MujocoTrajectoryController::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "MuJoCo FollowJointTrajectory action server READY");
  }

private:
  rclcpp_action::Server<FollowJT>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const FollowJT::Goal> goal)
  {
    if (goal->trajectory.points.empty()) {
      RCLCPP_WARN(get_logger(), "Rejected empty trajectory");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleJT>)
  {
    RCLCPP_WARN(get_logger(), "Trajectory cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleJT> goal_handle)
  {
    std::thread{std::bind(&MujocoTrajectoryController::execute, this, goal_handle)}.detach();
  }

  // 🚨 NO RENDERING OR mj_step HERE
  void execute(const std::shared_ptr<GoalHandleJT> goal_handle)
  {
    const auto& traj = goal_handle->get_goal()->trajectory;
    rclcpp::Time start_time = this->now();

    for (const auto& point : traj.points)
    {
      if (goal_handle->is_canceling())
      {
        goal_handle->canceled(std::make_shared<FollowJT::Result>());
        return;
      }

      rclcpp::Time target_time = start_time + point.time_from_start;

      // ⏳ wait until execution time
      while (rclcpp::ok() && this->now() < target_time)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      // 🎯 apply joint positions
      for (size_t i = 0; i < traj.joint_names.size(); ++i)
      {
        d->qpos[joint_map[traj.joint_names[i]]] = point.positions[i];
      }
    }

    goal_handle->succeed(std::make_shared<FollowJT::Result>());
    RCLCPP_INFO(get_logger(), "Trajectory execution complete");
  }
};

// =====================================================
// Main
// =====================================================
int main(int argc, char** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: mujoco_follow_joint_trajectory scene.xml\n";
    return 1;
  }

  rclcpp::init(argc, argv);

  load_mujoco(argv[1]);
  build_joint_map();

  auto node = std::make_shared<MujocoTrajectoryController>();

  // ---------------- Main loop ----------------
  while (rclcpp::ok() && !glfwWindowShouldClose(window))
  {
    rclcpp::spin_some(node);

    mj_step(m, d);

    mjrRect viewport = {0, 0, 1200, 900};
    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // ---------------- Cleanup ----------------
  mjr_freeContext(&con);
  mjv_freeScene(&scn);
  mj_deleteData(d);
  mj_deleteModel(m);
  glfwTerminate();

  rclcpp::shutdown();
  return 0;
}


