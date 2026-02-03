cmake_minimum_required(VERSION 3.8)
project(mujoco_cpp_pkg)

# =========================
# Compiler options
# =========================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Core dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# =========================
# ROS messages
# =========================
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# =========================
# GLFW
# =========================
find_package(glfw3 REQUIRED)

# =========================
# MuJoCo SDK (manual)
# =========================
if(NOT DEFINED ENV{MUJOCO_DIR})
  message(FATAL_ERROR "MUJOCO_DIR environment variable is not set")
endif()

set(MUJOCO_DIR $ENV{MUJOCO_DIR})

if(NOT EXISTS "${MUJOCO_DIR}/lib/libmujoco.so")
  message(FATAL_ERROR "libmujoco.so not found in ${MUJOCO_DIR}/lib")
endif()

add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES
  IMPORTED_LOCATION "${MUJOCO_DIR}/lib/libmujoco.so"
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include"
)

message(STATUS "Using MuJoCo from ${MUJOCO_DIR}")

# =========================
# Include directories
# =========================
include_directories(include)

# ============================================================
# 🚀 Executable 1: MuJoCo Follow Joint Trajectory
# ============================================================
add_executable(mujoco_follow_joint_trajectory
  src/mujoco_follow_joint_trajectory.cpp
)

ament_target_dependencies(mujoco_follow_joint_trajectory
  rclcpp
  rclcpp_action
  control_msgs
)

target_link_libraries(mujoco_follow_joint_trajectory
  mujoco
  glfw
)

# ============================================================
# 🤖 Executable 2: UR10e logic node (no MoveIt yet)
# ============================================================
add_executable(ur10e_moveit_robot1
  src/ur10e_moveit_robot1.cpp
)

ament_target_dependencies(ur10e_moveit_robot1
  rclcpp
  geometry_msgs
)

# ============================================================
# 📦 Installation
# ============================================================
install(TARGETS
  mujoco_follow_joint_trajectory
  ur10e_moveit_robot1
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
