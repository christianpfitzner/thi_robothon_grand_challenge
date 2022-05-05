# THI Robothon Grand Challenge

## Getting Started

1. [Install ROS 2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

2. Create a roboTHIx workspace. If it exists already, skip it.
```
mkdir -p ~/ws_roboTHIx/src
```
3. Pull the package, install dependencies, compile and source the workspace
```
cd ws_roboTHIx/src
git clone https://github.com/roboTHIx/thi_robothon_grand_challenge.git
rosdep update
rosdep install --ignore-src --from-paths src -y -r
colcon build --symlink-install
source install/setup.bash
```

## Using Examples
- `example_opencv_colorpicker` shows contours in an image
```
ros2 run thi_robothon_grand_challenge example_opencv_colorpicker_node 
```

- `example_opencv_contours` filters out a color section in an image
```
ros2 run thi_robothon_grand_challenge example_opencv_contours_node 
```
