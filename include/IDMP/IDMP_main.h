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

#ifndef IDMP_MAIN_H
#define IDMP_MAIN_H


#include "../IDMP/IDMP.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <idmp_ros/GetDistanceGradient.h>
#include <mutex>

namespace IDMP_ros
{
    class IDMPNode
    {

    public:
        IDMPNode( ros::NodeHandle& nh );

    private:
        void camInfoCB(const sensor_msgs::CameraInfoConstPtr& info_msg, const int camId);
        void pclCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);
        bool queryMap(idmp_ros::GetDistanceGradientRequest &req, idmp_ros::GetDistanceGradientResponse &res);

        sensor_msgs::PointCloud2 ptsToPcl(std::vector<float> &pts, std::vector<double> *queryRes, std::string frame);
        pcl::PointCloud<pcl::PointXYZRGB> ptsToPcl(std::vector<float> &pts, std::vector<uint8_t> &col, std::string frame);
        Eigen::Matrix4f lookupTf(const std::string& target_frame, const std::string& source_frame, const ros::Time& time, const ros::Duration timeout);


    private:
        ros::NodeHandle m_nh;

        ros::Publisher m_pclPub;
        ros::Publisher m_distanceSlice;

        ros::Subscriber pclSub;
        ros::Subscriber camInfoSub;
        std::vector<ros::Subscriber> camInfoSubs;
        std::string m_worldFrameId; // the map frame

        int numCams;
        std::vector<std::string> camTransforms;

        image_geometry::PinholeCameraModel m_model;

        ros::ServiceServer m_query_svc;

        tf2_ros::Buffer m_tf2Buffer;
        tf2_ros::TransformListener m_tf2Listener;

        IDMP_ros::IDMP idmp;

        std::vector<double> pRes;
        bool filtOutl;
        bool pubPcl;
        std::mutex mtx;
    };

}

#endif
