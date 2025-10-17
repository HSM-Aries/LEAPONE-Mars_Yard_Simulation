# LEAPONE Mars Surface Simulation

## Pre-requisites
Ubuntu 22.04: https://releases.ubuntu.com/jammy/

ROS2 Humble: https://docs.ros.org/en/humble/index.html  

Gazebo Harmonic or up

## Sensor used

IMU  
Realsense D435i Camera  
3D Lidar  


## Installation
1.  Clone this repository

    ```
    git clone https://github.com/shreyaspatel3010/aries.git
    ```

3. Source the Workspace

   ```
   cd aries
   colcon build
   source install/setup.bash
   ```

4. Install dependencies
   
	  ```
	  rosdep install --from-paths src -r -y
	  ```
> [!IMPORTANT]
> Change mesh location from common_properties.xacro which is in urdf files

## Launch Details

### Rviz + GUI
  ```
  ros2 launch aries display.launch.xml
  ```
### Gazebo + Rviz
  ```
  ros2 launch aries my_robot.launch.xml
  ```

## Teleop Function


  ```
  cd /src/aries/script
  python3 teleop_keyboard.py
  ```
> [!NOTE]
> read logs of teleop to understand operating commands.

# Result


https://github.com/user-attachments/assets/4f6c97ae-8522-4dd3-ba43-20f34e15a712


