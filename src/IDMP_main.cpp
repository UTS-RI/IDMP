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
    , m_it(nh)
    , m_tf2Listener(m_tf2Buffer)
    {
        IDMP_ros::IDMPParam idmpParams;
        nh.getParam("/idmp_rleng",idmpParams.rleng);
        nh.getParam("/idmp_tree_hl_min",idmpParams.tree_min_hl);
        nh.getParam("/idmp_tree_hl_max",idmpParams.tree_max_hl);
        nh.getParam("/idmp_tree_hl_clust",idmpParams.tree_clust_hl);
        nh.getParam("/idmp_tree_hl_init",idmpParams.tree_init_hl);
        nh.getParam("/idmp_map_scale",idmpParams.map_scale_param);
        nh.getParam("/idmp_obs_skip",idmpParams.obs_skip);
        nh.getParam("/idmp_dynamic",idmpParams.dynamic);
        nh.getParam("/idmp_fusion",idmpParams.fusion);
        nh.getParam("/idmp_dyn_tresh",idmpParams.dyn_tresh);
        nh.getParam("/idmp_fus_min",idmpParams.fus_min);
        nh.getParam("/idmp_fus_max",idmpParams.fus_max);
        nh.getParam("/idmp_filt_outl",filtOutl);
        nh.getParam("/idmp_pub_pcl",pubPcl);
        nh.getParam("/idmp_world_frame",m_worldFrameId);
        idmp.setParams(idmpParams);

        bool depthInput, dualCam;
        nh.getParam("/idmp_depth_input", depthInput);
        nh.getParam("/idmp_dual_cam", dualCam);

        if(depthInput) {
            std::string depthTopic, dual_depth_topic;
            image_transport::TransportHints hints("raw", ros::TransportHints(), nh);
            nh.getParam("/idmp_depth_topic", depthTopic);
            m_sub_depth = m_it.subscribeCamera(depthTopic, 1, &IDMPNode::depthImageCallback, this, hints);
            if(dualCam){
                nh.getParam("/idmp_dual_depth_topic", dual_depth_topic);
                m_sub_depth2 = m_it.subscribeCamera(dual_depth_topic, 1, &IDMPNode::depthImageCallback, this, hints);                
            }
        } else {
            std::string pclTopic, camInfoTopic;
            nh.getParam("/idmp_pcl_topic", pclTopic);
            nh.getParam("/idmp_caminfo_topic", camInfoTopic);
            pclSub = m_nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>(pclTopic,1, &IDMPNode::pclCB, this);
            camInfoSub = m_nh.subscribe<sensor_msgs::CameraInfo>(camInfoTopic,1, &IDMPNode::camInfoCB, this);
        }

        m_pclPub = m_nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>( "gp_pcl", 0 , true);
        m_query_svc = nh.advertiseService( "query_dist_field", &IDMPNode::queryMap, this );
        m_distanceSlice = nh.advertise<sensor_msgs::PointCloud2>("distances", 0);
    }

    std::vector<float> IDMPNode::fetchRobotTF(std::string frame, ros::Time time)
    {
        std::vector<float> pose_vec;
        tf::Transform tran;
        geometry_msgs::TransformStamped local_transformStamped;
        try {
            local_transformStamped = m_tf2Buffer.lookupTransform(m_worldFrameId, frame, time, ros::Duration(3.0));
            tf::transformMsgToTF( local_transformStamped.transform, tran );
            pose_vec.resize(12);
            pose_vec[0] = tran.getOrigin().x();
            pose_vec[1] = tran.getOrigin().y();
            pose_vec[2] = tran.getOrigin().z();

            pose_vec[3] = tran.getBasis()[0][0];
            pose_vec[4] = tran.getBasis()[1][0];
            pose_vec[5] = tran.getBasis()[2][0];

            pose_vec[6] = tran.getBasis()[0][1];
            pose_vec[7] = tran.getBasis()[1][1];
            pose_vec[8] = tran.getBasis()[2][1];

            pose_vec[9] = tran.getBasis()[0][2];
            pose_vec[10] = tran.getBasis()[1][2];
            pose_vec[11] = tran.getBasis()[2][2];
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        return pose_vec;
    }
    
    void IDMPNode::camInfoCB(const sensor_msgs::CameraInfoConstPtr& cam_info){
        m_model.fromCameraInfo(cam_info);
        IDMP_ros::camParam c( m_model, cam_info->width, cam_info->height);
        idmp.setCam(c);
        camInfoSub.shutdown();
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
        m_distanceSlice.publish(ptsToPcl(queryPoints, &pRes, m_worldFrameId));
        return true;
    }

    void IDMPNode::pclCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {
        static int cnt = 1;
        static std::vector<double> times;
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfCld(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtCld(new pcl::PointCloud<pcl::PointXYZRGB>);
        geometry_msgs::TransformStamped cldFrame;
        try {
            cldFrame = m_tf2Buffer.lookupTransform(m_worldFrameId, msg->header.frame_id, pcl_conversions::fromPCL(msg->header.stamp), ros::Duration(0.1));
        } catch(tf2::ExtrapolationException &e){
            ROS_ERROR_STREAM(e.what());
            return;
        } catch(tf2::LookupException &e){
            ROS_ERROR_STREAM(e.what());
            return;
        }
        Eigen::Matrix4f transf;

        pcl_ros::transformAsMatrix(cldFrame.transform, transf);
        
        pcl_ros::transformPointCloud(*msg,*transfCld, cldFrame.transform);
        if(filtOutl) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
            sor.setInputCloud(transfCld);
            sor.setMeanK(20);
            sor.setStddevMulThresh(1.0);
            sor.filter (*filtCld);
        }
        mtx.lock();

        auto start = std::chrono::high_resolution_clock::now();
        if(filtOutl) auto testCld = idmp.processFrame(*filtCld, transf);
        else auto testCld = idmp.processFrame(*transfCld, transf);
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

    void IDMPNode::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                                 const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        static int cnt = 0;
        cnt++;
        const bool is_u_short = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1;

        // Update camera model
        m_model.fromCameraInfo(info_msg);

        int rows = depth_msg->height;
        int cols = depth_msg->width;

        IDMP_ros::camParam c( m_model.fx(), m_model.fy(), m_model.cx(), m_model.cy(), cols, rows);
        idmp.setCam(c);

        //depth message usually does not have the correct frame embedded
        std::string frame;
        if(depth_msg->header.frame_id == "camera_1_depth_optical_frame") {
            frame = "camera_1_depth_frame";
        } else if(depth_msg->header.frame_id == "camera_1_depth_frame") {
            frame = "camera_1_depth_frame";
        } else if(depth_msg->header.frame_id == "camera_2_depth_optical_frame") {
            frame = "camera_2_depth_frame";
        } else if(depth_msg->header.frame_id == "camera_2_depth_frame") {
            frame = "camera_2_depth_frame";
        } else if(depth_msg->header.frame_id == "camera_depth_optical_frame") {
            frame = "camera_depth_optical_frame";
        } else {
            return;
        }
        std::vector<float> pose = fetchRobotTF(frame, depth_msg->header.stamp);
        float *input_z = 0;
        cv::Mat gp_img( cols, rows, CV_32FC1 );

        if (is_u_short) {
            const uint16_t* depth_row_base = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
            input_z = reinterpret_cast<float*>(gp_img.data);
            int row_step = depth_msg->step / sizeof(uint16_t);
            #pragma omp parallel for
            for (int v = 0; v < (int)rows; ++v)
            {
                const uint16_t* depth_row = depth_row_base + row_step*v;
                for (int u = 0; u < (int)cols; ++u )
                {
                    float depth = static_cast<float>( depth_row[u] );
                    // Missing points denoted by NaNs
                    if (!depth_image_proc::DepthTraits<uint16_t>::valid(depth)) {
                        depth = 0;
                    }

                    int k = rows * u + v;       // transposed image
                    input_z[ k ] = depth_image_proc::DepthTraits<uint16_t>::toMeters( depth );
                }
            }
        }
        else {
            cv::Mat debug_image( rows, cols, CV_16UC1 );

            const float* depth_row_base = reinterpret_cast<const float*>(&depth_msg->data[0]);
            input_z = reinterpret_cast<float*>(gp_img.data);
            int row_step = depth_msg->step / sizeof(float);
            #pragma omp parallel for
            for (int v = 0; v < (int)rows; ++v )
            {
                const float* depth_row = depth_row_base + row_step*v;
                for (int u = 0; u < (int)cols; ++u )
                {
                    float depth = depth_row[u];

                    // Missing points denoted by NaNs
                    if (!depth_image_proc::DepthTraits<float>::valid(depth)) {
                        depth = 0;
                    }
                    //gp_img.at<float>(u, v) = depth;
                    int k = rows * u + v;       // transposed image
                    input_z[ k ] = depth;
                    debug_image.at<uint16_t>(v, u) = //static_cast<uint16_t>( depth * 10000 );
                            depth_image_proc::DepthTraits<uint16_t>::fromMeters( depth );
                }
            }

        }

        int N_in = rows * cols;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfCld(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtCld(new pcl::PointCloud<pcl::PointXYZRGB>);
        *filtCld = idmp.createPcl( input_z, N_in, pose );

        geometry_msgs::TransformStamped cldFrame;
        try {
            cldFrame = m_tf2Buffer.lookupTransform(m_worldFrameId, frame, depth_msg->header.stamp, ros::Duration(1.0));
        } catch(tf2::ExtrapolationException &e){
            ROS_ERROR_STREAM(e.what());
            return;
        }
        Eigen::Matrix4f transf;
        pcl_ros::transformAsMatrix(cldFrame.transform, transf);
        pcl_ros::transformPointCloud(*filtCld,*transfCld, cldFrame.transform);

        // pcl::VoxelGrid<pcl::PointXYZRGB> filt;
        // filt.setInputCloud(transfCld);
        // filt.setLeafSize(filtVox, filtVox, filtVox);
        // filt.filter(*filtCld);

        mtx.lock();
        auto start = std::chrono::high_resolution_clock::now();
        auto testCld = idmp.processFrame(*transfCld, transf);
        std::cout <<cnt<<" "<< (std::chrono::high_resolution_clock::now()-start).count()*1E-6 << std::endl << std::flush;
        testCld.header.frame_id = m_worldFrameId;
        mtx.unlock();
        std::vector<float> allpts;
        std::vector<uint8_t> allcols;
        idmp.getAllPoints(allpts, allcols);
        m_pclPub.publish(ptsToPcl(allpts, allcols, m_worldFrameId));
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