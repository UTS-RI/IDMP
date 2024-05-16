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
#include <IDMP/IDMP.h>
#include <IDMP/params.h>
#include <chrono>
#include <thread>
#include <fstream>

void ticc(int mode=0) {
    static std::chrono::_V2::system_clock::time_point t_start;
    
    if (mode==0)
        t_start = std::chrono::high_resolution_clock::now();
    else {
        auto t_end = std::chrono::high_resolution_clock::now();
        std::cout << (t_end-t_start).count()*1E-6 <<"\n" << std::flush;
    }
}
void toci(int i) {std::cout << i << " "; ticc(1); }

namespace IDMP_ros
{

    IDMP::IDMP():t(0), frustum(0) {
        init();
    }

    IDMP::IDMP(IDMPParam par):t(0), frustum(0), setting(par) {
        init();
    }

    IDMP::IDMP(IDMPParam par, camParam c):t(0), frustum(0), setting(par), cam(c) {
        init();
    }

    IDMP::~IDMP() {
        reset();
    }

    void IDMP::init() {
        pose_tr.resize(3);
        pose_R.resize(9);
        frustum = new IDMP_ros::Frustum(cam.width, cam.height, cam.cx, cam.cy, cam.fx, cam.fy);
        cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    void IDMP::setParams(IDMPParam par) {
        setting = par;
    }

    void IDMP::reset() {
        if (t != 0) {
            delete t;
            t = 0;
        }

        activeSet.clear();

        return;
    }

    void IDMP::setCam(camParam c) {
        cam = c;
        if(cam.fx != 0) {
            frustum = new IDMP_ros::Frustum(cam.width, cam.height, cam.cx, cam.cy, cam.fx, cam.fy);
        } else {
            frustum = new IDMP_ros::Frustum(cam.width, cam.height, cam.cam_model);
        }
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB> IDMP::createPcl(float *dataz, int N, std::vector<float> &pose) {
        pcl::PointCloud<pcl::PointXYZRGB> cld;
        if (dataz == 0 || N < 1)
            return cld;

        if (pose.size() != 12)
            return cld;

        std::copy(pose.begin(), pose.begin() + 3, pose_tr.begin());
        std::copy(pose.begin() + 3, pose.end(), pose_R.begin());
        int cols = cam.width / setting.obs_skip; // x
        int rows = cam.height / setting.obs_skip; // y
        cld.header.frame_id = "base_link";
        cld.width = rows*cols;
        cld.height = 1;
        cld.points.resize(rows*cols);
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int u = r*setting.obs_skip;
                int v = c*setting.obs_skip;
                int k = u*cam.height+v;
                int idx = c*rows+r;
                if ((dataz[k] < DEPTH_MAX_RANGE) && (dataz[k] > DEPTH_MIN_RANGE)){
                    float x_loc = dataz[k] * (float(u)-cam.cx)/cam.fx;
                    float y_loc = dataz[k] * (float(v)-cam.cy)/cam.fy;
                    pcl::PointXYZRGB p;
                    
                    p.x = x_loc;//pose_R[0] * x_loc + pose_R[3] * y_loc + pose_R[6] * dataz[k] + pose_tr[0];
                    p.y = y_loc;//pose_R[1] * x_loc + pose_R[4] * y_loc + pose_R[7] * dataz[k] + pose_tr[1];
                    p.z = dataz[k];//pose_R[2] * x_loc + pose_R[5] * y_loc + pose_R[8] * dataz[k] + pose_tr[2];
                    cld.points[idx] = p;
                }
            }
        }
        return cld;
    }
    
    pcl::PointCloud<pcl::PointXYZRGB> IDMP::processFrame(pcl::PointCloud<pcl::PointXYZRGB> &cld, Eigen::Matrix4f pose) {
        pcl::PointCloud<pcl::PointXYZRGB> tempCld;
        if(t != 0 && (setting.dynamic || setting.fusion)) {
            IDMP tempGp;
            auto params = setting;
            params.oneshot=true;
            tempGp.setParams(setting);
            tempGp.setCam(this->cam);
            tempGp.update(cld);
            frustum->calcPlanes(pose.cast<double>());
            Eigen::Vector3d camNormal = (pose.cast<double>() * Eigen::Vector4d(0, 0, 1,0)).head<3>();
            IDMP_ros::vecNode3 nodes;
            t->getAllFrustumNodes(nodes, *frustum);

            kd_tree::Static3dTree<point_cloud::PointCloud<point_type::Point3f>> stat_3d_tree;
            if(setting.fusion) {
                // std::vector<point_type::Point3f> cldPts(cld.points.size());
                for(int i = 0; i<cld.points.size();i++){
                    stat_3d_tree.add(point_type::Point3f(cld.points[i].x,cld.points[i].y,cld.points[i].z));
                }
                stat_3d_tree.build();
            }
            if(nodes.size() > 0) {
                std::vector<float> testPts(nodes.size()*3);
                #pragma omp parallel for
                for(int i = 0; i < nodes.size(); i++){
                    int i3=i*3;
                    testPts[i3] = nodes[i]->getPosX();
                    testPts[i3+1] = nodes[i]->getPosY();
                    testPts[i3+2] = nodes[i]->getPosZ();
                }

                std::vector<double> res(nodes.size()*8);
                tempGp.test(testPts.data(), 3, nodes.size(), res.data());
                std::vector<bool> cldDel(cld.points.size(),false);
                #pragma omp parallel for
                for(int i = 0; i<nodes.size(); i++){
                    int i8 = i*8;
                    if(setting.dynamic && res[i8]>setting.dyn_tresh) {  //Dynamic
                        //check normal direction
                        if(camNormal.dot(Eigen::Vector3d(res[i8+1],res[i8+2],res[i8+3])) < 1){
                            point_type::Point3f searchPoint(nodes[i]->getPosX(), nodes[i]->getPosY(), nodes[i]->getPosZ());
                            std::vector<size_t> indices;
                            std::vector<float> squared_distances;
                            dynamic_3d_tree.nearestKSearch(searchPoint, 1, indices, squared_distances);
                            dynamic_3d_tree.remove(indices[0]);
                            t->MarkRemoval(nodes[i]);
                            continue;
                        }
                    }
                    if(setting.fusion && res[i8] > setting.fus_min && res[i8] < setting.fus_max) { //move point
                        pcl::PointXYZRGB p;
                        p.x = nodes[i]->getPosX() - res[i8]*res[i8+1];
                        p.y = nodes[i]->getPosY() - res[i8]*res[i8+2];
                        p.z = nodes[i]->getPosZ() - res[i8]*res[i8+3];
                        p.r = nodes[i]->getColorR();
                        p.g = nodes[i]->getColorG();
                        p.b = nodes[i]->getColorB();

                        point_type::Point3f searchPoint(nodes[i]->getPosX(), nodes[i]->getPosY(), nodes[i]->getPosZ());
                        std::vector<size_t> indices;
                        std::vector<float> squared_distances;
                        dynamic_3d_tree.nearestKSearch(searchPoint, 1, indices, squared_distances);
                        dynamic_3d_tree.remove(indices[0]);
                        t->MarkRemoval(nodes[i]);
                        indices.clear();
                        squared_distances.clear();
                        stat_3d_tree.nearestKSearch(searchPoint, 1, indices, squared_distances);
                        cld.points[indices[0]].x = nodes[i]->getPosX() - res[i8]*0.5*res[i8+1];
                        cld.points[indices[0]].y = nodes[i]->getPosY() - res[i8]*0.5*res[i8+2];
                        cld.points[indices[0]].z = nodes[i]->getPosZ() - res[i8]*0.5*res[i8+3];
                    } else if(setting.fusion && res[i8] < setting.fus_min){
                        // point_type::Point3f searchPoint(nodes[i]->getPosX(), nodes[i]->getPosY(), nodes[i]->getPosZ());
                        // std::vector<size_t> indices;
                        // std::vector<float> squared_distances;
                        // stat_3d_tree.nearestKSearch(searchPoint, 1, indices, squared_distances);
                        // cldDel[indices[0]] = true;
                    }
                }
                t->RemoveMarked();
                for(int i = cldDel.size()-1; i>=0; i--) {
                    if(cldDel[i]) cld.points.erase(cld.points.begin()+i);
                }
            }
            stat_3d_tree.reset();
            update(cld);
        } else {
            update(cld);
        }
        return tempCld;
    }

    bool IDMP::update(float *dataz, int N, std::vector<float> &pose) {
        auto cld = createPcl(dataz, N, pose);
        update(cld);
	    return true;
    }

    int IDMP::update(pcl::PointCloud<pcl::PointXYZRGB> &cld) {
        if (t == 0) {
            tree_param tp = {setting.tree_min_hl, setting.tree_max_hl, setting.tree_init_hl, setting.tree_clust_hl};
            t = new OcTree(Point3<float>(0.0, 0.0, 0.0), tp);
        }
        std::vector<point_type::Point3f> addedPts;
        int pointCnt = 0;
        for(int i=0; i < cld.points.size(); i++) {
            
            std::shared_ptr<Node3> p(new Node3(IDMP_ros::Point3<float>(cld.points[i].x, cld.points[i].y, cld.points[i].z), IDMP_ros::Point3<uint8_t>(cld.points[i].r, cld.points[i].g, cld.points[i].b)));
            std::unordered_set<OcTree *> vecInserted;

            bool succeeded = false;
            // #pragma omp critical
            if (!t->IsNotNew(p)) {
                succeeded = t->Insert(p, vecInserted);
                if (succeeded) {
                    pointCnt++;
                    addedPts.emplace_back(cld.points[i].x, cld.points[i].y, cld.points[i].z);
                    cloud->points.push_back(cld.points[i]);
                    if (t->IsRoot() == false) {
                        t = t->getRoot();
                    }
                }
            }
            for (auto it = vecInserted.begin(); it != vecInserted.end(); it++) {
                activeSet.insert(*it);
            }
        }
        if(setting.oneshot) {
            for(auto p:addedPts){
                static_3d_tree.add(p);
            }
            static_3d_tree.build();
        } else{
            dynamic_3d_tree.add(addedPts);
        }
        updateGPs();
        return pointCnt;
    }
    
    void IDMP::updateGPs_kernel(int thread_idx,
                                    int start_idx,
                                    int end_idx,
                                    OcTree **nodes_to_update) {
        
        #pragma omp parallel for
        for (int i = start_idx; i < end_idx; ++i) {
            if (nodes_to_update[i] != 0) {
                Point3<float> ct = (nodes_to_update[i])->getCenter();
                point_type::Point3f searchPoint(ct.x, ct.y, ct.z); 
                std::vector<size_t> indices;
                float radius = (nodes_to_update[i])->getHalfLength() * setting.rleng;
                if(setting.oneshot) {
                    static_3d_tree.radiusSearch(searchPoint, radius, indices);
                } else {
                    dynamic_3d_tree.radiusSearch(searchPoint, radius, indices);
                }
                if (indices.size() > 0) {
                    Eigen::Matrix<double,3,Eigen::Dynamic> trainMat;
                    trainMat.resize(Eigen::NoChange,indices.size());
                    for(int j = 0; j<indices.size(); j++) {
                        point_type::Point3f cp = dynamic_3d_tree[indices[j]];
                        trainMat(0,j) = cp.x;
                        trainMat(1,j) = cp.y;
                        trainMat(2,j) = cp.z;
                    }
                    std::shared_ptr<IDMP_ros::gp> gp(new IDMP_ros::gp(setting.map_scale_param));
                    gp->train_new(trainMat);
                    (nodes_to_update[i])->Update(gp);
                }
            }
        }
    }

    void IDMP::updateGPs() {

        std::unordered_set<OcTree *> updateSet(activeSet);
        for(auto it = activeSet.begin(); it != activeSet.end(); it++) {

            Point3<float> ct = (*it)->getCenter();
            float l = (*it)->getHalfLength();
            // AABB3 searchbb(ct.x, ct.y, ct.z, setting.rleng * l);
            // std::vector<OcTree *> qs;
            // t->QueryNonEmptyLevelC(searchbb, qs);
            point_type::Point3f searchPoint(ct.x, ct.y, ct.z);
            std::vector<size_t> indices;
            float radius = (*it)->getHalfLength() * setting.rleng;
            if(setting.oneshot) {
                static_3d_tree.radiusSearch(searchPoint, radius, indices);
            } else {
                dynamic_3d_tree.radiusSearch(searchPoint, radius, indices);
            }

            if (indices.size() > 0) {
                for(auto idx:indices) {
                    point_type::Point3f cp = setting.oneshot ? static_3d_tree[idx] : dynamic_3d_tree[idx];
                    OcTree* cluster;
                    t->getContainingTreeC(IDMP_ros::Point3<float>(cp.x,cp.y,cp.z), cluster);
                    updateSet.insert(cluster);
                }
            }
            // if(qs.size() > 0) {
            //     for (auto itq = qs.begin(); itq != qs.end(); itq++) {
            //         updateSet.insert(*itq);
            //     }
            // }
        }
        int num_elements = updateSet.size();
        if (num_elements < 1)
            return;

        OcTree **nodes_to_update = new OcTree *[num_elements];
        int it_counter = 0;
        for (auto it = updateSet.begin(); it != updateSet.end(); ++it, ++it_counter) {
            nodes_to_update[it_counter] = *it;
        }
        updateGPs_kernel(0,0,num_elements, nodes_to_update);
        // clear active set once all the jobs for update are done.
        activeSet.clear();

        return;
    }

    void IDMP::test_kernel(int thread_idx,
                               int start_idx,
                               int end_idx,
                               float *x,
                               double *res,
				int *hit_cnt) {

        #pragma omp parallel for
        for (int i = start_idx; i < end_idx; ++i) {

            int k3 = 3 * i;
            EVectorX xt(3);
            xt << x[k3], x[k3 + 1], x[k3 + 2];

            int k8 = 8 * i;

            

            
            res[k8 + 4] = 1.0 ; // variance of sdf value
            // query Cs
            
            //AABB3 searchbb(xt(0), xt(1), xt(2), setting.tree_clust_hl * 50.0);
            // std::vector<OcTree *> quads;
            // std::vector<float> sqdst;
            // t->QueryNonEmptyLevelC(searchbb, quads, sqdst);
            // if (sqdst.size() > 0) {
            //     // get clostest cluster
            //     auto result = std::min_element(sqdst.begin(), sqdst.end());
            //     std::shared_ptr<gp> gp = quads[std::distance(sqdst.begin(), result)]->getGP();
            //     if (gp != nullptr) {
            //         gp->testSinglePoint_new(xt, res[k8], &res[k8 + 1], &res[k8 + 4]);
            //     }
            // }
            
            point_type::Point3f searchPoint(xt(0), xt(1), xt(2)); 
            std::vector<size_t> indices;
            std::vector<float> dists;
            OcTree* nearestCluster = nullptr;
            if(setting.oneshot) {
                static_3d_tree.nearestKSearch(searchPoint, 1, indices, dists);
            } else {
                dynamic_3d_tree.nearestKSearch(searchPoint, 1, indices, dists);
            }
            if(indices.size()>0){
                point_type::Point3f cp = setting.oneshot ? static_3d_tree[indices[0]] : dynamic_3d_tree[indices[0]];
                t->getContainingTreeC(IDMP_ros::Point3<float>(cp.x,cp.y,cp.z), nearestCluster);
            }
            if(nearestCluster != nullptr) { 
                std::shared_ptr<gp> gp = nearestCluster->getGP();
                if (gp != nullptr) {
                    gp->testSinglePoint_new(xt, res[k8], &res[k8 + 1], &res[k8 + 4]);
                }
            }
            #ifndef RevertingKernel
            float invert_a = 1/setting.map_scale_param;
            res[k8] = -(invert_a)*log(abs(res[k8]));
            //normalize gradients
            double gradLen = -1 * sqrt(pow(res[k8+1], 2) + pow(res[k8+2], 2) + pow(res[k8+3], 2)); 
            #else
            float invert_a = 1/sqrt(setting.map_scale_param);
            double oldres = res[k8];
            res[k8] = sqrt(abs(-2*invert_a*invert_a*log(abs(res[k8]))));
            //normalize gradients
            double gradLen = sqrt(pow(res[k8+1], 2) + pow(res[k8+2], 2) + pow(res[k8+3], 2)); 
            #endif

            if(gradLen != 0){
                res[k8 + 1]/=gradLen;
                res[k8 + 2]/=gradLen;
                res[k8 + 3]/=gradLen;
            } else {
                res[k8 + 1]=0;
                res[k8 + 2]=0;
                res[k8 + 3]=0;
            }
        }
    }

    bool IDMP::test(float *x, int dim, int leng, double *res) {
        if (x == 0 || dim != 3 || leng < 1)
            return false;
        if(t == 0) {
            ROS_ERROR_STREAM("GP IS NOT TRAINED!");
            return false;
        }
	    std::vector<int> hit_cnt(1, 0);
        test_kernel(0,0, leng, x, res, &hit_cnt[0]);
        return true;
    }

    void IDMP::getAllPoints(std::vector<float> &pos) {
        pos.clear();

        if (t == 0)
            return;

        std::vector<std::shared_ptr<Node3> > nodes;
        t->getAllChildrenNonEmptyNodes(nodes);

        int N = nodes.size();
        if (N > 0) {
            pos.resize(3 * N);
            for (int j = 0; j < N; j++) {
                int j3 = 3 * j;
                pos[j3] = nodes[j]->getPosX();
                pos[j3 + 1] = nodes[j]->getPosY();
                pos[j3 + 2] = nodes[j]->getPosZ();
            }
        }
        return;
    }

    void IDMP::getAllPoints(std::vector<float> &pos, std::vector<uint8_t> &col) {
        pos.clear();
        col.clear();

        if (t == 0)
            return;

        std::vector<std::shared_ptr<Node3> > nodes;
        t->getAllChildrenNonEmptyNodes(nodes);

        int N = nodes.size();
        if (N > 0) {
            pos.resize(3 * N);
            col.resize(3 * N);
            for (int j = 0; j < N; j++) {
                int j3 = 3 * j;
                pos[j3] = nodes[j]->getPosX();
                pos[j3 + 1] = nodes[j]->getPosY();
                pos[j3 + 2] = nodes[j]->getPosZ();
                col[j3] = nodes[j]->getColorR();
                col[j3 + 1] = nodes[j]->getColorG();
                col[j3 + 2] = nodes[j]->getColorB();
            }
        }
        return;
    }

    void IDMP::getAllPoints(pcl::PointCloud<pcl::PointXYZRGB> &cld) {
        if (t == 0)
            return;

        std::vector<std::shared_ptr<Node3> > nodes;
        t->getAllChildrenNonEmptyNodes(nodes);

        if (nodes.size() > 0) {
            cld.width = nodes.size();
            cld.height = 1;
            cld.points.resize(nodes.size());
            #pragma omp parallel for
            for (int j = 0; j < nodes.size(); j++) {
                cld.points[j].x = nodes[j]->getPosX();
                cld.points[j].y = nodes[j]->getPosY();
                cld.points[j].z = nodes[j]->getPosZ();
                cld.points[j].r = nodes[j]->getColorR();
                cld.points[j].g = nodes[j]->getColorG();
                cld.points[j].b = nodes[j]->getColorB();
            }
        }
        // cld = *cloud;
    }
}
