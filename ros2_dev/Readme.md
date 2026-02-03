ROS 2 Humble Dev Container with VNC
Overview

This document explains how to set up a ROS 2 Humble development container on Windows using Docker VS Code and VNC.
The setup uses WSL2 and Docker Desktop.
These steps are tested and known to work.
Follow them in order from start to finish.

Requirements

You need Windows 10 or Windows 11.
Docker Desktop must be installed and running.
Docker Desktop must use the WSL2 backend.
VS Code must be installed.
The Dev Containers extension must be installed in VS Code.
The Docker extension must be installed in VS Code.

Step 1 Create the project folder

Create a new empty folder on Windows.
Example location

C:\Users\your_name\ros2_vnc_workspace


Open this folder in VS Code.

Step 2 Create the devcontainer folder

Inside the project folder create a folder named

.devcontainer


Inside the devcontainer folder create two files

Dockerfile
devcontainer.json


The folder structure must look like this

ros2_vnc_workspace
.devcontainer
Dockerfile
devcontainer.json


Do not create extra folders.

Step 3 Disable bash history mounting

This step is very important.

Open VS Code user settings JSON.
Add the following line.

"dev.containers.mountBashHistory": false


Save the file.
Close VS Code completely.
Make sure VS Code is not running in the background.

Step 4 Dockerfile

Create the file

.devcontainer/Dockerfile


Paste the following content.

FROM ros:humble-ros-base

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000

RUN if ! id -u ${USER_UID} >/dev/null 2>&1; then \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME}; \
    fi

RUN apt-get update && apt-get install -y sudo && \
    echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

USER ubuntu
WORKDIR /workspace

RUN sudo apt update && sudo apt install -y git curl gnupg lsb-release

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


Save the file.

Step 5 devcontainer.json

Create the file

.devcontainer/devcontainer.json


Paste the following content.

{
  "name": "ros2-humble-vnc",
  "build": {
    "dockerfile": "Dockerfile"
  },
  "remoteUser": "ubuntu",
  "runArgs": ["--privileged"],
  "workspaceMount": "source=${localWorkspaceFolder},target=/workspace,type=bind",
  "workspaceFolder": "/workspace",
  "features": {
    "ghcr.io/devcontainers/features/desktop-lite:1": {
      "webPort": 6080,
      "vncPort": 5901,
      "password": "noPassword"
    }
  },
  "forwardPorts": [6080, 5901]
}


Save the file.

Step 6 Clean Docker state

Open PowerShell.

Run the following commands.

docker system prune -af
docker volume prune -f


Restart Docker Desktop.

Step 7 Build the container

Open VS Code.
Open the folder ros2_vnc_workspace.

Press Ctrl Shift P.
Select Rebuild and Reopen in Container.
Wait for the build to finish.

Step 8 Open the desktop

Open a web browser.
Go to

http://localhost:6080


You should see a Linux desktop environment.

Step 9 Test ROS

Open a terminal inside the desktop.

Run

rviz2


RViz should start successfully.

Step 10 Create a ROS workspace

In the container terminal run

cd /workspace
mkdir -p ros2_ws/src
cd ros2_ws


Optional test package

cd src
ros2 pkg create my_first_pkg --build-type ament_cmake

Step 11 Run rosdep

First time only

sudo rosdep init
rosdep update


Install dependencies

cd /workspace/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

Step 12 Build the workspace
cd /workspace/ros2_ws
colcon build
source install/setup.bash

Optional Install Gazebo

For Gazebo Classic

sudo apt install -y gazebo ros-humble-gazebo-ros-pkgs
gazebo


For modern Gazebo

sudo apt install -y gz-tools2 libgz-sim7
gz sim

Final Result

You now have a working ROS 2 Humble dev container.
The workspace is persistent on Windows.
RViz works through VNC.
Gazebo can be installed when needed.
This setup can be reused for future projects.

Workspace Git setup

All Git commands must be run from the workspace root directory.

/workspace/ros2_dev


Do not run submodule commands from inside the src folder.

If the workspace is not already a Git repository run this once.

cd /workspace/ros2_dev
git init

Adding Universal Robots description as a submodule

From the workspace root run:

git submodule add https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git src/Universal_Robots_ROS2_Description


This clones the repository into the src folder and registers it as a submodule.

Adding Franka description as a submodule

From the workspace root run:

git submodule add https://github.com/frankarobotics/franka_description.git src/franka_description


Both robot description repositories are now tracked as submodules.

Initializing submodules

To make sure all submodules are fully checked out run:

git submodule update --init --recursive


This is also required when cloning this workspace on a new machine.

Ignoring build artifacts

ROS build folders must not be committed.

Create a file named .gitignore in the workspace root with the following content.

build/
install/
log/

Committing the submodules

From the workspace root run:

git add .gitmodules .gitignore src/Universal_Robots_ROS2_Description src/franka_description
git commit -m "Add robot description repositories as submodules"


This commits the submodule references and configuration.

Building after adding submodules

After adding new packages always install dependencies and rebuild.

cd /workspace/ros2_dev
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash



Now onward to working with urdf and xacro  
Key Concepts Used
1. XACRO as Source of Truth

All robot geometry and structure is defined in XACRO

URDF files are generated artifacts, not manually edited

XACRO enables modular composition and reuse

2. UR10e Integration

UR10e model is defined via a custom macro (ur10e_macro.xacro)

Meshes are resolved via:

package://ur_description/meshes/...


Dependency on ur_description is declared in package.xml

Macro supports prefixing for multi-robot setups

3. Critical XACRO Fix: Parse-Time Variables

A key issue encountered was:

name 'prefix' is not defined


Root cause:
XACRO evaluates some expressions at parse time, not macro-instantiation time.

Fix applied:

<xacro:property name="prefix" value=""/>


This guarantees prefix exists during parsing and prevents hard-to-debug failures.

4. Franka Gripper Integration

The Franka gripper was added as a standalone macro and attached to the UR10e tool0 frame.

Key fixes made to the gripper XACRO:

Aligned macro parameters with how the macro is instantiated

Removed undefined helper macros (e.g. link_with_sc)

Ensured all parent links exist before joints reference them

Used consistent mesh package resolution

Ensured URDF safety (check_urdf passes)

The gripper is attached via:

<xacro:franka_hand
  connected_to="robot1_tool0"
  arm_id="robot1_arm"
  gazebo="true"/>

5. Full Robot Composition

The top-level robot XACRO (ur10e_franka_table.xacro) composes:

World frame

UR10e robot

Franka gripper

Table

Camera with Gazebo sensor plugin

All components are instantiated explicitly, ensuring the generated URDF is complete.

Converting XACRO → URDF

To generate a plain URDF:

cd /workspace/ros2_dev
source install/setup.bash

xacro src/my_robot_description/urdf/ur10e_franka_table.xacro \
  > src/my_robot_description/urdf/ur10e_franka_table.urdf


Validate:

check_urdf src/my_robot_description/urdf/ur10e_franka_table.urdf


Expected result:

Successfully parsed URDF file

Build & Install
colcon build
source install/setup.bash


Installed resources are available at:

$(ros2 pkg prefix my_robot_description)/share/my_robot_description/


moveit installtion

sudo apt update
sudo apt install -y ros-humble-moveit





moveit_resources

Add moveit_resources (ros2 branch) as a Git submodule

We’ll add only once, cleanly, under src/.

1️⃣ Go to workspace root
cd /workspace/ros2_dev


Make sure you are at the git repo root (same place as .gitmodules).

2️⃣ Add the submodule (ROS 2 branch)
git submodule add -b ros2 https://github.com/moveit/moveit_resources.git src/moveit_resources


This will:

Clone the repo

Track the ros2 branch

Register it in .gitmodules

Verify:

git status


You should see:

.gitmodules modified

src/moveit_resources added

3️⃣ Initialize & update (safe even if already cloned)
git submodule update --init --recursive

4️⃣ Install dependencies
rosdep install --from-paths src --ignore-src -r -y


This is important — moveit_resources has dependencies.

5️⃣ Build the workspace
colcon build
source install/setup.bash

6️⃣ Verify the required package exists

This is the key one your Franka hand needs:

ros2 pkg prefix moveit_resources_panda_description


Also sanity-check meshes:

ls $(ros2 pkg prefix moveit_resources_panda_description)/share/moveit_resources_panda_description/meshes/visual


You must see:

finger.dae
hand.dae

