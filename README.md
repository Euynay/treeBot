# ROS Forest Tree Species Importance Measurement Device

Welcome to the repository for the "ROS Forest Tree Species Importance Measurement Device" project. This project utilizes the Robot Operating System (ROS) to develop a device capable of measuring the importance of forest tree species through a series of integrated software components and hardware interactions.

## Overview

This project aims to automate the process of assessing the importance of different tree species in a forest, which is crucial for ecological studies and forest management. By leveraging ROS, we have created a system that includes a mobile platform (husky robot), sensors for data collection, and machine learning algorithms for species identification.

## ROS Workspace Structure

The ROS workspace is organized as follows:

```
my_ws/
├── src/
   ├── husky/               # Husky robot model files
   │   ├── husky_control/   # Control models for the robot's chassis and motors
   │   ├── husky_gazebo/    # Simulation environment setup with forest map
   │   ├── husky_navigation/# Navigation algorithms and launch files
   ├── loam_velodyne/       # SLAM algorithm for map building from LiDAR data
   ├── cloudpoi_process/    # Point cloud data processing for tree measurement
   ├── darknet_ros/         # Real-time object detection using the Darknet framework
   └── wx_ros_launch/       # Communication with the mini-program and message listening
```

## Getting Started

To set up and run the project, follow these steps:

1. **Build the ROS Workspace:**
   ```bash
   $ cd my_ws
   $ catkin_make
   $ source devel/setup.bash
   ```

2. **Set up the Husky URDF:**
   ```bash
   $ export HUSKY_URDF_EXTRAS=$(rospack find husky_description)/urdf/empty.urdf
   ```

3. **Launch the Husky Simulation:**
   ```bash
   $ roslaunch husky_gazebo husky_playpen.launch
   ```

4. **Start the Mapping and Navigation Nodes:**
   ```bash
   $ roslaunch loam_velodyne loam_velodyne.launch
   $ roslaunch octomap_server octomap_mapping.launch
   $ roslaunch octomap_server octomap_server.launch
   ```

5. **Process Point Cloud Data:**
   ```bash
   $ rosrun cloudpoi_process pointcloud_processor_node
   ```

6. **Start the Tree Species Identification:**
   ```bash
   $ rqt_image_view  # To view camera feed
   $ roslaunch darknet_ros darknet_ros.launch
   ```

## Mini-program Usage

After logging in, users can initiate the robot from the mapping page and control its movement to complete the mapping process. Once the map is created, it will be automatically saved. Users can then navigate using the newly created map by setting start and end points, and the robot will autonomously move to the specified location. Additionally, users can start the identification and measurement of data in the measurement section.

## Contributing

We welcome contributions to this project! If you have ideas or fixes, please submit a pull request or create an issue.

## License

This project is open-source.

---

For more detailed information and specific instructions, please refer to the individual README files within each directory or contact the project maintainers.
