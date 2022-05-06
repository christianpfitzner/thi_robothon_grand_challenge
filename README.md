# THI Robothon Grand Challenge
Package of the Technische Hochschule Ingolstadt for the competiton [Robothon Grand Challenge 2022](https://robothon-grand-challenge.com/).
<br/>
<br/>

## Getting Started
1. [Install ROS 2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html). If done, you can skip it.

2. Create the roboTHIx workspace:
  ```
  mkdir -p ~/ws_roboTHIx/src
  ```
  
3. Pull the package, install dependencies, compile and source the workspace:
  ```
  cd ws_roboTHIx/src
  git clone https://github.com/roboTHIx/thi_robothon_grand_challenge.git
  rosdep update
  rosdep install --ignore-src --from-paths src -y -r
  colcon build --symlink-install
  source install/setup.bash
  ```
<br/>

## Used Hardware
Installation of the packages for each hardware component is descriped on the linked github pages.

  ### Camera
  - [`usb_cam`](https://github.com/ros-drivers/usb_cam) For testing the OpenCV 2D nodes with built-in webcams etc.
  - [`Orbbec Astra Mini`](https://github.com/roboTHIx/ros_astra_camera) To get a RGB image and a point cloud
  
  ### Robotarm
  - [`UR 10 e`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/galactic) To manipulate the task board
  - [`Weiss WSG 50 Gripper`](https://github.com/nalt/wsg50-ros-pkg) Gripper with coustom made finger tips
<br/>


## How To Use
  ### Robot Vision
  #### Parameters
  - `params_vision.yaml` All used parameters for the vision nodes
  <br/>
  
  #### RGB
  - `example_opencv_colorpicker` Example node to filter out a color section in an image
  ```
  ros2 run thi_robothon_grand_challenge example_opencv_colorpicker_node 
  ```
  - `example_opencv_contours` Example node to show contours in an image
  ```
  ros2 run thi_robothon_grand_challenge example_opencv_contours_node 
  ```
  <br/>
  
  ### Point Cloud
  
<br/>

  ### Robot Manipulation
  #### Parameters
  - `params_manipulation.yaml` All used parameters for the manipulation nodes
  <br/>
  
  #### Robot Arm Control
  <br/>
  
  #### Gripper Control
  <br/>
  
<br/>
