/*
 *    IDMP - Interactive Distance Field Mapping and Planning to Enable Human-Robot Collaboration
 *    Copyright (C) 2024 Usama Ali
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License v3 as published by
 *    the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License v3 for more details.
 *
 *    You should have received a copy of the GNU General Public License v3
 *    along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.html.
 *
 *    Authors: Usama Ali <usama.ali@thws.de>
 *             Adrian Mueller <adrian.mueller@thws.de>
 *             Lan Wu <Lan.Wu-2@uts.edu.au>
 */

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl/io/ply_io.h>

int main( int argc, char** argv )
{
    ros::init( argc, argv, "dist_eval" );
    ros::NodeHandle node_handle;

    ROS_INFO_STREAM("Running eval node");

    pcl::PointCloud<pcl::PointXYZRGB> gt;

    pcl::PLYReader reader;

    reader.read("/home/usama/Workspace/ros/utsWs/bags/ladyCow/cow_and_lady_gt.ply", gt);
    auto m_pclPub = node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>( "/camera/depth_registered/points", 1 , true);
    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    gt.header.frame_id = "base_link";
    m_pclPub.publish(gt);
    spinner.spin();
    return 1;
}