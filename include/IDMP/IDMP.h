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

#ifndef __GPIS_MAP3__H__
#define __GPIS_MAP3__H__

#include "gp.h"
#include "octree.h"
#include "params.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Point.h>
#include "../kdtree/dynamic_3d_tree.hpp"
#include "../kdtree/static_3d_tree.hpp"
#include "../kdtree/point_cloud.hpp"
#include "../kdtree/point_type.hpp"
#include <image_geometry/pinhole_camera_model.h>


namespace IDMP_ros {
    typedef struct camParam_ {
        float fx;
        float fy;
        float cx;
        float cy;
        int width;
        int height;
        image_geometry::PinholeCameraModel cam_model;
        //initial params. should get overwritten by CameraInfo
        camParam_() {
            width = 848;
            height = 480;
            fx = 422.6003112792969; // 570.9361;
            fy = 422.6003112792969; // 570.9361;
            cx = 426.28948974609375;// 307;
            cy = 237.70343017578125; //240;
        }

        camParam_(float fx_, float fy_, float cx_, float cy_, float w_, float h_) : fx(fx_), fy(fy_), cx(cx_), cy(cy_),
                                                                                    width(w_), height(h_) {}
        camParam_(image_geometry::PinholeCameraModel cam_model_, float w_, float h_) : fx(0), fy(0), cx(0), cy(0),
                                                                                    width(w_), height(h_), cam_model(cam_model_) {}
    } camParam;

    typedef struct IDMPParam_ {
        int obs_skip;     // use every 'skip'-th pixel
        float map_scale_param;
        float tree_min_hl;
        float tree_max_hl;
        float tree_init_hl;
        float tree_clust_hl;
        float rleng;
        bool dynamic;
        bool fusion;
        float dyn_tresh;
        float fus_min;
        float fus_max;
        bool oneshot;

        IDMPParam_() {
            obs_skip = DEPTH_SKIP;
            map_scale_param = DEFAULT_MAP_SCALE_PARAM;
            tree_min_hl = DEFAULT_TREE_MIN_HALFLENGTH;
            tree_max_hl = DEFAULT_TREE_MAX_HALFLENGTH;
            tree_init_hl = DEFAULT_TREE_INIT_ROOT_HALFLENGTH;
            tree_clust_hl = DEFAULT_TREE_CLUSTER_HALFLENGTH;
            rleng = DEFAULT_RLENG;
            dynamic = true;
            fusion = true;
            dyn_tresh = 0.05;
            fus_min = 0.005;
            fus_max = 0.05;
            oneshot = false;
        }

        IDMPParam_(IDMPParam_ &par) {
            obs_skip = par.obs_skip;
            map_scale_param = par.map_scale_param;
            tree_min_hl = par.tree_min_hl;
            tree_max_hl = par.tree_max_hl;
            tree_init_hl = par.tree_init_hl;
            tree_clust_hl = par.tree_clust_hl;
            rleng = par.rleng;
            dynamic = par.dynamic;
            fusion = par.fusion;
            dyn_tresh = par.dyn_tresh;
            fus_min = par.fus_min;
            fus_max = par.fus_max;
            oneshot = par.oneshot;            
        }
        
    } IDMPParam;

    class IDMP {
    protected:
        IDMPParam setting;
        camParam cam;

        OcTree *t;
        std::unordered_set<OcTree *> activeSet;
        std::vector<IDMP_ros::Frustum> frustum;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        kd_tree::Dynamic3dTree<point_cloud::PointCloud<point_type::Point3f>>dynamic_3d_tree;
        kd_tree::Static3dTree<point_cloud::PointCloud<point_type::Point3f>>static_3d_tree;

        void init();

        void updateGPs();

        std::vector<float> pose_tr;
        std::vector<float> pose_R;

    public:
        IDMP();

        IDMP(IDMPParam par);

        IDMP(IDMPParam par, camParam c);
    
        void setParams(IDMPParam par, int numCams);

        pcl::PointCloud<pcl::PointXYZRGB> createPcl(float *dataz, int N, std::vector<float> &pose);

        pcl::PointCloud<pcl::PointXYZRGB> processFrame(pcl::PointCloud<pcl::PointXYZRGB> &cld, std::vector<Eigen::Matrix4f> camPoses);

        ~IDMP();

        void reset();

        void getAllPoints(std::vector<float> &pos);

        void getAllPoints(std::vector<float> &pos, std::vector<uint8_t> &col);
    
        void getAllPoints(pcl::PointCloud<pcl::PointXYZRGB> &cld);

        //Update for depth input
        bool update(float *dataz, int N, std::vector<float> &pose);
        //Update for pointcloud input
        int update(pcl::PointCloud<pcl::PointXYZRGB> &cld);

        bool test( float *x, int dim, int leng, double *res );

        void setCam(camParam c, int id);

        inline float getOcTreeResolution( )
        { return t->getMinHalfLength() * 2; }

        inline const IDMPParam & getSettings () const
        { return setting; }

    private:
        void test_kernel(int thread_idx,
                         int start_idx,
                         int end_idx,
                         float *x,
                         double *res,
                         int *hit_cnt);

        void updateGPs_kernel(int thread_idx,
                              int start_idx,
                              int end_idx,
                              OcTree **nodes_to_update);
    };

}

#endif
