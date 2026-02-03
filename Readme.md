# ROS 2 MuJoCo Workspace

## Overview
## This repository is still under working phase in organizing things otherwise the codes included in it for mujoco-ros2-moveit and moveit/movegroup_interface cpp API code will work perfectly
This repository contains a ROS 2 Humble workspace designed to support robot description development simulation using MuJoCo and future integration with motion planning frameworks.

The workspace is intended to be used inside a Docker based development container on Windows using WSL2 and VS Code Dev Containers.

The focus of this project is

Robot modeling using URDF and XACRO  
Physics based simulation using MuJoCo  
ROS 2 control and action interfaces  
A clean reproducible development environment  

MoveIt is intentionally not finalized in this repository due to GUI instability inside containerized environments. The setup however is compatible with MoveIt for future use.

---

## Host System Requirements

Windows 10 or Windows 11  
WSL2 enabled  
Docker Desktop with WSL backend enabled  
Visual Studio Code  
VS Code Dev Containers extension  
VS Code Docker extension  

---

## Repository Structure

workspace_root
├── .devcontainer
│ ├── Dockerfile
│ └── devcontainer.json
├── .vscode
│ └── settings.json
├── ros2_dev
│ ├── src
│ │ ├── mujoco_cpp_pkg
│ │ ├── my_robot_description
│ │ └── other ROS packages
│ ├── build
│ ├── install
│ └── log
└── README.md


The ros2_dev directory is a standard ROS 2 workspace.  
All development happens inside the src directory.

---

## Development Container Setup

### Step 1 Create Workspace Folder

Create a folder on your Windows machine

C:\Users\your_name\ros2_mujoco_workspace


Open this folder in VS Code.

---

### Step 2 Devcontainer Files

Inside the workspace create a folder named

.devcontainer


Inside this folder create two files

Dockerfile
devcontainer.json


---

### Dockerfile

FROM osrf/ros:humble-desktop-full

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000

RUN if ! id -u ${USER_UID} >/dev/null 2>&1; then
groupadd --gid ${USER_GID} ${USERNAME} &&
useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME};
fi

RUN apt-get update && apt-get install -y sudo &&
echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} &&
chmod 0440 /etc/sudoers.d/${USERNAME}

USER ubuntu
WORKDIR /workspace

RUN sudo apt update
RUN sudo apt install -y git curl

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


---

### devcontainer.json

{
"name": "ros2-humble-mujoco",
"build": {
"dockerfile": "Dockerfile"
},
"remoteUser": "ubuntu",
"runArgs": [
"--privileged"
],
"workspaceMount": "source=${localWorkspaceFolder},target=/workspace,type=bind",
"workspaceFolder": "/workspace",
"features": {
"ghcr.io/devcontainers/features/desktop-lite:1": {
"webPort": 6080,
"vncPort": 5901,
"password": "noPassword"
}
},
"forwardPorts": [6080]
}


---

### Step 3 Build Container

Open VS Code Command Palette  
Select Rebuild and Reopen in Container  
Wait for the build to complete  

Once finished open a browser and go to

http://localhost:6080


You should see a Linux desktop.

---

## ROS 2 Workspace Setup

Inside the container terminal

cd /workspace
mkdir -p ros2_dev/src
cd ros2_dev


Install dependencies

rosdep update
rosdep install --from-paths src --ignore-src -r -y


Build workspace

colcon build
source install/setup.bash


---

## Robot Description Workflow

### XACRO as Source of Truth

All robot models are written in XACRO.  
URDF files are generated artifacts and never edited manually.

XACRO allows

Modular robot composition  
Reusable macros  
Prefixing for multi robot setups  

---

### UR10e Integration

UR10e is included using macros from the UR description package.

Meshes are resolved using

package://ur_description/meshes


Prefix support is implemented to allow multiple instances.

A critical fix was required to avoid XACRO parse time errors.

<xacro:property name="prefix" value=""/>


This ensures variables exist during parse time.

---

### Franka Gripper Integration

The Franka hand is added as a standalone macro.

Key fixes applied

Correct macro parameters  
Removed undefined helper macros  
Ensured parent links exist  
Verified URDF validity  

The gripper is attached to the tool frame of the robot.

---

### Full Robot Composition

The top level robot XACRO includes

World frame  
UR10e robot  
Franka gripper  
Table  
Optional camera  

This ensures the generated URDF is complete and valid.

---

### Generate URDF from XACRO

cd /workspace/ros2_dev
source install/setup.bash

xacro src/my_robot_description/urdf/ur10e_franka_table.xacro \

src/my_robot_description/urdf/ur10e_franka_table.urdf


Validate

check_urdf src/my_robot_description/urdf/ur10e_franka_table.urdf


---

## MuJoCo Installation

MuJoCo is installed manually inside the container.

### Step 1 Download MuJoCo

cd ~
wget https://github.com/google-deepmind/mujoco/releases/download/3.1.1/mujoco-3.1.1-linux-x86_64.tar.gz
tar -xvf mujoco-3.1.1-linux-x86_64.tar.gz


Expected directory

~/mujoco-3.1.1


---

### Step 2 Environment Variables

Add to bashrc

export MUJOCO_DIR=$HOME/mujoco-3.1.1
export LD_LIBRARY_PATH=$MUJOCO_DIR/lib:$LD_LIBRARY_PATH


Reload

source ~/.bashrc


Verify

echo $MUJOCO_DIR
ls $MUJOCO_DIR/lib/libmujoco.so


---

### Step 3 System Dependencies

sudo apt update
sudo apt install -y
libglfw3-dev
libglew-dev
libgl1-mesa-dev
libxinerama-dev
libxcursor-dev
libxi-dev
libxrandr-dev


---

### Step 4 Test MuJoCo Standalone

cd ~/mujoco-3.1.1/bin
./simulate ../model/humanoid.xml


A visible simulation window must appear.

---

## MuJoCo ROS Package

The package `mujoco_cpp_pkg` contains

A MuJoCo follow joint trajectory action node  
Integration hooks for ROS 2 control  
CMake configuration for MuJoCo SDK  

The CMakeLists uses imported MuJoCo library targets and links against GLFW.

---

## MoveIt Status

MoveIt is installed using

sudo apt install -y ros-humble-moveit


However the MoveIt Setup Assistant GUI is unstable inside containerized desktop environments and frequently segfaults.

For this reason MoveIt configuration is postponed.  
The robot description is fully compatible with MoveIt and can be used later on a native Linux system.

---

## Git Notes

This repository does not currently track robot description repositories as submodules.

Submodules can be added later once the workspace stabilizes.

Build directories are ignored

build
install
log


---

## Final Notes

This workspace provides

A clean ROS 2 Humble environment  
MuJoCo physics simulation  
Modular robot description workflow  
A reproducible container based setup  

The project is intended to grow toward

Multi robot simulation  
Advanced control  
Planning integration  
Reinforcement learning experiments