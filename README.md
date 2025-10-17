# Aries-Mars Rover workspace discription and instructions

## Pre-requisites

ROS2 Jazzy/Humble  
Gazebo Harmonic or up

## Sensor used

imu  
Realsense D435i Camera  
Lidar  


## installation
1.  for clone this repo into your home directiory using terminal

    ```
    git clone https://github.com/shreyaspatel3010/aries.git
    ```

3. source workspace

   ```
   cd aries
   colcon build
   source install/setup.bash
   ```

4. command for auto detect dependency and install to run file
   
	  ```
	  rosdep install --from-paths src -r -y
	  ```
> [!IMPORTANT]
> Change mesh location from common_properties.xacro which is in urdf files

## Launch detaiis

### Rviz + gui
  ```
  ros2 launch aries display.launch.xml
  ```
### Gazebo + Rviz
  ```
  ros2 launch aries my_robot.launch.xml
  ```

## Teleop
from src/aries/script run teleop_keyboard.py in treminal
  ```
  python3 teleop_keyboard.py
  ```
> [!NOTE]
> read logs of teleop to understand operating commands.

# Result


https://github.com/user-attachments/assets/4f6c97ae-8522-4dd3-ba43-20f34e15a712


