# Robotic Arm ISRO Project 🦾(Need to change this)

This repository contains a ROS 2 Humble workspace for the simulation and motion planning of a Universal Robots UR10e robotic arm. The project is configured for use with MoveIt 2 and includes a custom URDF with an added bowl model at the base of the robot for simulated interaction tasks.

  <img width="631" height="472" alt="image" src="https://github.com/user-attachments/assets/b96c0ed4-ca2d-4d50-bd8c-5071c016aab5" />

  <img width="631" height="472" alt="image" src="https://github.com/user-attachments/assets/70c2f612-89c2-442b-af8a-3282bb46f6a6" />




## Features
* **Robot**: Universal Robots UR10e
* **Framework**: ROS 2 Humble Hawksbill
* **Motion Planning**: Configured for MoveIt 2
* **Customization**: Includes a static `bowl.stl` model attached to the robot's `base_link`.

---
## Prerequisites
Before you begin, ensure you have the following installed:
* Ubuntu 22.04
* [ROS 2 Humble Desktop Full](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* [MoveIt 2 for ROS 2 Humble](https://moveit.ros.org/install-moveit2/binary/)
    ```bash
    sudo apt install ros-humble-moveit
    ```
* Colcon and other ROS 2 tools
    ```bash
    sudo apt install python3-colcon-common-extensions ros-humble-ros2-control ros-humble-ros2-controllers
    ```

---
## 🛠️ Setup and Installation

Follow these steps to set up the workspace.

**1. Clone the Repository**
```bash
git clone [https://github.com/Nithish-Kumar19181/Robotic_arm_ISRO.git](https://github.com/Nithish-Kumar19181/Robotic_arm_ISRO.git) robotic_arm_ws
cd robotic_arm_ws
```

**2. Install Dependencies**
Install all the required package dependencies from the `src` folder.
```bash
rosdep install --from-paths src --ignore-src -r -y
```

**3. Build the Workspace**
Build all the packages using `colcon`.
```bash
colcon build --symlink-install
```

**4. Source the Workspace**
Before running any ROS 2 commands, you must source the workspace's setup file.
```bash
source install/setup.bash
```
**Note**: You need to run this `source` command in every new terminal you open.

---
## 🚀 Usage

Once the workspace is built and sourced, you can perform the following actions.

### **1. Visualize the Robot in RViz**
To view the UR10e model with the custom bowl and test its joints, run the following launch command:
```bash
ros2 launch ur_description display.launch.py
```
This will open RViz and a GUI with sliders to control the robot's joints.

### **2. Configure with MoveIt Setup Assistant**
The MoveIt Setup Assistant requires a final, static URDF file, not a XACRO template. Follow these steps to generate the file and launch the assistant.

**Step A: Generate the URDF File**
Run the `xacro` command to convert the robot's XACRO description into a single URDF file. This command resolves all arguments and creates the file in the `/tmp` directory.
```bash
ros2 run xacro xacro src/Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro ur_type:=ur10e > /tmp/ur10e_for_moveit.urdf
```

**Step B: Launch the MoveIt Setup Assistant**
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

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
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

**Step C: Load the URDF**
In the Setup Assistant's start screen, click **"Browse"** and navigate to the file you just created: `/tmp/ur10e_for_moveit.urdf`.

The robot model will now load successfully, and you can proceed with generating the MoveIt configuration package.
