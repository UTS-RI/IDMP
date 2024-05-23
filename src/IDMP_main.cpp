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

#include <IDMP/IDMP_main.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <depth_image_proc/depth_traits.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <fstream>
#include <chrono>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace IDMP_ros
{

    IDMPNode::IDMPNode( ros::NodeHandle& nh )
    : m_nh( nh )
    , m_worldFrameId( "base_link" )
    , m_tf2Listener(m_tf2Buffer)
    {
        IDMP_ros::IDMPParam idmpParams;
        std::string pclTopic;
        XmlRpc::XmlRpcValue camData; //2d Vector not implemented so we have to parse it ourself yay
        nh.getParam("/idmp_rleng",idmpParams.rleng);
        nh.getParam("/idmp_tree_hl_min",idmpParams.tree_min_hl);
        nh.getParam("/idmp_tree_hl_max",idmpParams.tree_max_hl);
        nh.getParam("/idmp_tree_hl_clust",idmpParams.tree_clust_hl);
        nh.getParam("/idmp_tree_hl_init",idmpParams.tree_init_hl);
        nh.getParam("/idmp_map_scale",idmpParams.map_scale_param);
        nh.getParam("/idmp_dynamic",idmpParams.dynamic);
        nh.getParam("/idmp_fusion",idmpParams.fusion);
        nh.getParam("/idmp_dyn_tresh",idmpParams.dyn_tresh);
        nh.getParam("/idmp_fus_min",idmpParams.fus_min);
        nh.getParam("/idmp_fus_max",idmpParams.fus_max);
        nh.getParam("/idmp_filt_outl",filtOutl);
        nh.getParam("/idmp_pub_pcl",pubPcl);
        nh.getParam("/idmp_world_frame",m_worldFrameId);
        nh.getParam("/idmp_pcl_topic", pclTopic);
        nh.getParam("/idmp_caminfo_topic", camData);
        numCams = camData.size();
        idmp.setParams(idmpParams, numCams);
        ROS_INFO_STREAM("Starting IDMP with:");
        ROS_INFO_STREAM("Dynamic:\t"<<idmpParams.dynamic);
        ROS_INFO_STREAM("Fusion:\t\t"<<idmpParams.fusion);

        

        ROS_INFO_STREAM("IDMP input:\t"<<pclTopic);
        ROS_INFO_STREAM("Number of Cameras:\t"<<numCams);
        pclSub = m_nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>(pclTopic,1, &IDMPNode::pclCB, this);
        for(int i = 0; i < numCams; i++) {
            ROS_INFO_STREAM("IDMP Camera "<<i<<":\t"<<camData[i][0]<<"\tregistered to transform:\t"<<camData[i][1]);
            camInfoSubs.push_back(m_nh.subscribe<sensor_msgs::CameraInfo>(camData[i][0],1, boost::bind(&IDMPNode::camInfoCB, this, _1, i)));
            camTransforms.push_back(camData[i][1]);
        }

        m_pclPub = m_nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>( "gp_pcl", 0 , true);
        m_query_svc = nh.advertiseService( "query_dist_field", &IDMPNode::queryMap, this );
        m_distanceSlice = nh.advertise<sensor_msgs::PointCloud2>("distances", 0);
    }
    
    void IDMPNode::camInfoCB(const sensor_msgs::CameraInfoConstPtr& cam_info, const int camId){
        m_model.fromCameraInfo(cam_info);
        IDMP_ros::camParam c( m_model, cam_info->width, cam_info->height);
        idmp.setCam(c, camId);
        camInfoSubs[camId].shutdown();
    }

    bool IDMPNode::queryMap(idmp_ros::GetDistanceGradientRequest &req, idmp_ros::GetDistanceGradientResponse &res) {        
        std::vector<float> queryPoints(req.points.begin(), req.points.end());
        int N_pts = queryPoints.size()/3;
        pRes.clear();
        pRes.resize( N_pts * 8, 0 );
        auto start = std::chrono::high_resolution_clock::now();
        mtx.lock();
        idmp.test( queryPoints.data(), 3, N_pts, pRes.data() );
        mtx.unlock();
        std::cout << "Query: "<< (std::chrono::high_resolution_clock::now()-start).count()*1E-6 << std::endl << std::flush;
        res.stamp = ros::Time::now();
        res.distances.resize(N_pts, 0);
        res.gradients.resize(N_pts*3, 0);
        res.in_bounds.resize(N_pts, 1);
        // #pragma omp parallel for
        for (int index = 0; index < N_pts; index++) {
            int k8 = index * 8;
            res.distances[index] = static_cast<double>(pRes[k8]);
            res.gradients[(index*3)] = static_cast<double>(pRes[k8 + 1]);
            res.gradients[(index*3)+1] = static_cast<double>(pRes[k8 + 2]);
            res.gradients[(index*3)+2] = static_cast<double>(pRes[k8 + 3]);
        }
        
        // uncomment to publish queried distance field
        // m_distanceSlice.publish(ptsToPcl(queryPoints, &pRes, m_worldFrameId));
        return true;
    }

    Eigen::Matrix4f IDMPNode::lookupTf(const std::string& target_frame, const std::string& source_frame, const ros::Time& time, const ros::Duration timeout) {
        geometry_msgs::TransformStamped transfMsg;
        Eigen::Matrix4f transfMat;
        try {
            transfMsg = m_tf2Buffer.lookupTransform(target_frame, source_frame, time, timeout);
        } catch(std::exception &e){
            ROS_ERROR_STREAM(e.what());
            return Eigen::Matrix4f::Zero();
        }
        pcl_ros::transformAsMatrix(transfMsg.transform, transfMat);
        return transfMat;
    }

    void IDMPNode::pclCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {
        static int cnt = 1;
        static std::vector<double> times;
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfCld(new pcl::PointCloud<pcl::PointXYZRGB>);
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtCld(new pcl::PointCloud<pcl::PointXYZRGB>);

        if(msg->header.frame_id != m_worldFrameId) { //transform pointcloud into world frame if not already done
            geometry_msgs::TransformStamped cldFrame;
            try {
                cldFrame = m_tf2Buffer.lookupTransform(m_worldFrameId, msg->header.frame_id, pcl_conversions::fromPCL(msg->header.stamp), ros::Duration(0.1));
            } catch(std::exception &e){
                ROS_ERROR_STREAM(e.what());
                return;
            } 
            pcl_ros::transformPointCloud(*msg,*transfCld, cldFrame.transform);
        } else {
            *transfCld = *msg;
        }

        if(filtOutl) { //perform outlier filtering
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
            sor.setInputCloud(transfCld);
            sor.setMeanK(20);
            sor.setStddevMulThresh(1.0);
            sor.filter (*filtCld);
        }
        std::vector<Eigen::Matrix4f> camPoses;
        for(auto tfFrame:camTransforms) {
            camPoses.push_back(lookupTf(m_worldFrameId, tfFrame, pcl_conversions::fromPCL(msg->header.stamp), ros::Duration(0.1)));
        }
        mtx.lock();
        auto start = std::chrono::high_resolution_clock::now();
        if(filtOutl) auto testCld = idmp.processFrame(*filtCld, camPoses);
        else auto testCld = idmp.processFrame(*transfCld, camPoses);
        times.push_back((std::chrono::high_resolution_clock::now()-start).count()*1E-6);
        std::cout <<cnt<<" "<< std::accumulate(times.begin(), times.end(), 0.0)/times.size() << std::endl << std::flush;

        if(pubPcl) {
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            idmp.getAllPoints(cloud);
            cloud.header.frame_id = m_worldFrameId;
            m_pclPub.publish(cloud);
        }
        mtx.unlock();
        cnt++;
    }

    pcl::PointCloud<pcl::PointXYZRGB> IDMPNode::ptsToPcl(std::vector<float> &pts, std::vector<uint8_t> &col, std::string frame){
        pcl::PointCloud<pcl::PointXYZRGB> pcl;
        pcl.header.frame_id = frame;
        pcl.width = pts.size()/3;
        pcl.height = 1;
        pcl.points.resize(pts.size()/3);
        #pragma omp parallel for
        for(int i = 0; i < pts.size()/3; i++) {
            int i3 = i*3;
            pcl::PointXYZRGB p;
            p.x = pts[i3]; 
            p.y = pts[i3 + 1];
            p.z = pts[i3 + 2];
            p.r = col[i3];
            p.g = col[i3 + 1];
            p.b = col[i3 + 2];
            pcl.points[i] = p;
        }
        return pcl;
    }

        
    sensor_msgs::PointCloud2 IDMPNode::ptsToPcl(std::vector<float> &pts, std::vector<double> *queryRes, std::string frame) {
        bool useDist = !(queryRes==NULL);
        sensor_msgs::PointCloud2 pcl_msg;
        //Modifier to describe what the fields are.
        sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
        
        if(useDist) {
            modifier.setPointCloud2Fields(4,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::PointField::FLOAT32);
        } else {
            modifier.setPointCloud2Fields(4,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::PointField::FLOAT32);
        }
        //Msg header
        pcl_msg.header = std_msgs::Header();
        pcl_msg.header.stamp = ros::Time::now();
        pcl_msg.header.frame_id = frame;

        pcl_msg.height = 1;
        pcl_msg.width = pts.size()/3;
        pcl_msg.is_dense = true;

        //Total number of bytes per poins
        pcl_msg.point_step = 16;//(useDist ? 16 : 12);
        
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
        pcl_msg.data.resize(pcl_msg.row_step);  
        //Iterators for PointCloud msg
        sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "intensity");

        //iterate over the message and populate the fields.      
        for(int i = 0; i < pts.size()/3; i++) {
            *iterX = pts[i*3];
            *iterY = pts[i*3+1];
            *iterZ = pts[i*3+2];

            // Increment the iterators
            ++iterX;
            ++iterY;
            ++iterZ;
            if(useDist) {
                *iterIntensity = (*queryRes)[i*8];
                ++iterIntensity;
            }
        }
        return pcl_msg;
    }
}

int main( int argc, char** argv )
{
    ros::init( argc, argv, "IDMP" );
    ros::NodeHandle node_handle;

    IDMP_ros::IDMPNode node( node_handle );
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
}