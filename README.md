# IDMP - Interactive Distance Field Mapping and Planning to Enable Human-Robot Collaboration

This is the code for IDMP, implementing a gaussian process based distance and gradient field algorithm.

 * [Project Page](https://uts-ri.github.io/IDMP/)
 * [Paper](https://arxiv.org/abs/2403.09988v1)
 * [Video](https://www.youtube.com/watch?v=NpbDjCqXyrs)
 
This codebase is implemented using ROS in both C++ and Python.

## Description

TODO: Maybe write something explaining what IDMP does briefly

## Dependencies

- Python3
- C++14
- ROS Noetic
- Eigen
- OpenMP (for multithreading. optional but strongly recommended)
- sensor-filters (`sudo apt install ros-noetic-sensor-filters`)
- point-cloud2-filters (`sudo apt install ros-noetic-point-cloud2-filters`)
- robot-body-filter (optional for removing the robot from the pointcloud. `sudo apt install ros-noetic-point-cloud2-filters`)
- Camera specific ros driver
    - [Azure kinect](https://github.com/microsoft/Azure_Kinect_ROS_Driver)
    - [Intel Realsense](https://github.com/IntelRealSense/realsense-ros)
    - [ZED 2i](https://github.com/stereolabs/zed-ros-wrapper)
- For Python tools:
    - numpy
    - OpenCV 4.9.0
    - Open3D (for evaluation only)

## Installation

IDMP is provided as catkin package

- clone the package into the src folder of your workspace
- run a `catkin build`
- resource your workspace

## Running IDMP
### Launch Files

We provide a few launch files to get started:

- realsense.launch: Launches the realsense driver, publishes the camera pose as TF and runs IDMP
- zed.launch: Launches the zed driver, publishes the camera pose as TF and runs IDMP
- kinect_UR5e.launch: Launches the Azure Kinect driver, an inoffical UR5e Ros driver, a filter node that filters the robot points and IDMP

You can run these with `roslaunch idmp_ros {name}.launch`.

The camera pose has to be published as a TF transform. This is already done in the launch files. To change the pose, adjust the corresponding parameters in the static transform publisher args

### Running IDMP standalone

If you do not want to use a launch file, you can also run the IDMP node standalone

`rosrun idmp_ros idmp`

Keep in mind that it needs the parameters of the config file in the parameter server at launch. You can lode these with `rosparam load {file}.yaml`.

These config also contains the topics of the input pointcloud, the camera intrinsics and the reference frame. Make sure to adjust these to your setup.

### Tools

We provide a set of tools to interact with IDMP for testing and demonstration purposes. You can run these with `rosrun idmp_ros {tool}.py

- calibPose: This can be used to calibrate the camera pose using an aruco marker. Adjust the topics and marker parameters for your setting and run the script. Make sure the determined pose is aligned properly and copy the output into your launchfile to replace the transform.
- queryTool: This creates an interactive marker and queries a distance field slice. It is then published and can be visualized in RVIZ. You can adjust the slice properties and make it move by setting the corresponding variables in the file.
- datasetTFPublisher: This publishes the pose topic found in the cow and lady dataset as TF
- evalTool: This is similar to the queryTool but it can load a groundtruth pointcloud and calculate real time metrics like RMSE
- speedMove: This is a basic implementation of obstacle avoidance with repulsive vectors by querying the distance field. This needs a robot driver to run and sends velocity commands to control the robot.
- The eval folder contains some tools used to evaluate our results for the publication

### Parameters

All IDMP parameters can be found as yaml files in the config folder. These need to be loaded into the parameter server before launching IDMP.

|Parameter|Description|Recommended Value|
|---|---|---|
|idmp_rleng|Cluster overlap factor for GP training|1.8|
|idmp_tree_hl_min|Octree half-length min|0.025|
|idmp_tree_hl_max|Octree half-length max|3.2|
|idmp_tree_hl_clust|Octree half-length of cluster|0.05|
|idmp_tree_hl_init|Initial octree half-length|1.6|
|idmp_map_scale|Kernel map scale|1024|
|idmp_obs_skip|Skip factor for depth filtering (Depth input only)|10|
|idmp_filt_outl|Enables outlier filtering for input|True|
|idmp_depth_input|Use depth image as input|False|
|idmp_dual_cam|Use dual camera setup (not implemented yet)|False|
|idmp_pub_pcl|Publish the pointcloud of the octree for visualization|True|
|idmp_depth_topic|Topic of depth input|"/camera_1/depth/image_raw"|
|idmp_dual_depth_topic|Topic of secondary depth input|""|
|idmp_world_frame|Reference frame (needs to be published as TF)|"base_link"|
|idmp_pcl_topic|Topic of input pointcloud|"/points_filt"|
|idmp_caminfo_topic|Topic of cameraInfo with intrinsic parameters of the depth sensor|"/depth/camera_info"|
|idmp_dynamic|Enables dynamic object mode|True|
|idmp_fusion|Enables local fusion|False|
|idmp_dyn_tresh|Dynamic movement treshold|0.08|
|idmp_fus_min|Fusion minimum treshold|0.0005|
|idmp_fus_max|Fusion maximum treshold|0.01|





















