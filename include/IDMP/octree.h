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

#ifndef __OCTREE_H_
#define __OCTREE_H_

#include <vector>
#include <unordered_set>
#include <memory>
#include <iostream>
#include <cstdint>
#include "strct.h"
#include "gp.h"
#include <Eigen/Core>
#include <cfloat>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/eigen.hpp>

namespace IDMP_ros
{
    class Plane {
    public:
        Plane(){}

        Plane(Eigen::Vector3d _normal, Eigen::Vector3d _point){
            normal = _normal;
            point = _point;
        }
        double dist(Eigen::Vector3d p){
            return normal.dot(p-point);
        }
        void transform(Eigen::Matrix4d pose){
            point = (pose * Eigen::Vector4d(point[0], point[1], point[2],1)).head<3>();
            normal = (pose * Eigen::Vector4d(normal[0], normal[1], normal[2],0)).head<3>();
        }
    private:
        Eigen::Vector3d normal; //Normal of plane
        Eigen::Vector3d point; //point of plane
    };

    class Frustum {
    public:
        Frustum() {
            empty = true;
        }

        Frustum(double width, double height, double cx, double cy, double fx, double fy){
            Eigen::Vector3d frustumPoints[8];
            frustumPoints[0] = Eigen::Vector3d((double(width)-cx)/fx, (double(height/2)-cy)/fy, 1); //right center
            frustumPoints[1] = Eigen::Vector3d(0,1,0); // down
            frustumPoints[2] = Eigen::Vector3d((double(width/2)-cx)/fx, (double(0)-cy)/fy, 1); // center top
            frustumPoints[3] = Eigen::Vector3d(1,0,0); // left
            frustumPoints[4] = Eigen::Vector3d((double(0)-cx)/fx, (double(height/2)-cy)/fy, 1); // left center
            frustumPoints[5] = Eigen::Vector3d(0,-1,0); // up
            frustumPoints[6] = Eigen::Vector3d((double(width/2)-cx)/fx, (double(height)-cy)/fy, 1); // center down
            frustumPoints[7] = Eigen::Vector3d(-1,0,0); // right
            for(int i = 0; i <4; i++) {
                int i2=i*2;
                origPlanes[i] = IDMP_ros::Plane(frustumPoints[i2].cross(frustumPoints[i2+1]).normalized(),frustumPoints[i2]);
            }
            empty = false;
        }

        Frustum(double width, double height, image_geometry::PinholeCameraModel camMod){
            Eigen::Vector3d frustumCorners[4];
            Eigen::Vector3d frustumPoints[8];
            if(camMod.distortionCoeffs().empty()) {
                cv2eigen(cv::Mat(camMod.projectPixelTo3dRay(cv::Point2d(0,0))), frustumCorners[0]); //right center
                cv2eigen(cv::Mat(camMod.projectPixelTo3dRay(cv::Point2d(0,height))), frustumCorners[1]); //right center
                cv2eigen(cv::Mat(camMod.projectPixelTo3dRay(cv::Point2d(width,height))), frustumCorners[2]); //right center
                cv2eigen(cv::Mat(camMod.projectPixelTo3dRay(cv::Point2d(width,0))), frustumCorners[3]); //right center
            } else {
                cv2eigen(cv::Mat(camMod.projectPixelTo3dRay(camMod.rectifyPoint(cv::Point2d(0,0)))), frustumCorners[0]); //right center
                cv2eigen(cv::Mat(camMod.projectPixelTo3dRay(camMod.rectifyPoint(cv::Point2d(0,height)))), frustumCorners[1]); //right center
                cv2eigen(cv::Mat(camMod.projectPixelTo3dRay(camMod.rectifyPoint(cv::Point2d(width,height)))), frustumCorners[2]); //right center
                cv2eigen(cv::Mat(camMod.projectPixelTo3dRay(camMod.rectifyPoint(cv::Point2d(width,0)))), frustumCorners[3]); //right center
            }
            for(int i = 0; i < 4; i++) {
                int i2 = i*2;
                frustumPoints[i2+1] = frustumCorners[(i+1)%4] - frustumCorners[i]; //nextCorner - corner
                frustumPoints[i2] = frustumCorners[i] + frustumPoints[i2+1]/2;  // middlepoint
            }
            for(int i = 0; i <4; i++) {
                int i2=i*2;
                origPlanes[i] = IDMP_ros::Plane(frustumPoints[i2].cross(frustumPoints[i2+1]).normalized(),frustumPoints[i2]);
            }
            empty = false;
        }

        void calcPlanes(Eigen::Matrix4d pose) {
            if(empty) return;
            for(int i = 0; i<4; i++) {
                planes[i]=origPlanes[i];
                planes[i].transform(pose);
            }
        }

        bool checkPoint(Eigen::Vector3d point) {
            if(empty) return false;
            for(int i = 0; i<4; i++) {
                double dist = planes[i].dist(point);
                if(dist > 0.05)return false;
            }
            return true;
        }

        inline bool checkEmpty() {return empty;}

    private:
    IDMP_ros::Plane origPlanes[4];
    IDMP_ros::Plane planes[4];
    bool empty;
    };

    class AABB3 {
        Point3<float> center;
        float halfLength;
        float halfLengthSq;
        float xmin;
        float xmax;
        float ymin;
        float ymax;
        float zmin;
        float zmax;

        Point3<float> ptNWF;
        Point3<float> ptNEF;
        Point3<float> ptSWF;
        Point3<float> ptSEF;
        Point3<float> ptNWB;
        Point3<float> ptNEB;
        Point3<float> ptSWB;
        Point3<float> ptSEB;
    public:
        AABB3() {
            halfLength = 0.0;
            halfLengthSq = 0.0;
            xmin = 0.0;
            xmax = 0.0;
            ymin = 0.0;
            ymax = 0.0;
            zmin = 0.0;
            zmax = 0.0;
        }

        AABB3(Point3<float> _center, float _halfLength) {
            center = _center;
            halfLength = _halfLength;
            halfLengthSq = halfLength * halfLength;
            xmin = center.x - halfLength;
            xmax = center.x + halfLength;
            ymin = center.y - halfLength;
            ymax = center.y + halfLength;
            zmin = center.z - halfLength;
            zmax = center.z + halfLength;
            ptNWF = Point3<float>(xmin, ymax, zmax);
            ptNEF = Point3<float>(xmax, ymax, zmax);
            ptSWF = Point3<float>(xmin, ymin, zmax);
            ptSEF = Point3<float>(xmax, ymin, zmax);
            ptNWB = Point3<float>(xmin, ymax, zmin);
            ptNEB = Point3<float>(xmax, ymax, zmin);
            ptSWB = Point3<float>(xmin, ymin, zmin);
            ptSEB = Point3<float>(xmax, ymin, zmin);
        }

        AABB3(float x, float y, float z, float _halfLength) {
            center = Point3<float>(x, y, z);
            halfLength = _halfLength;
            halfLengthSq = halfLength * halfLength;
            xmin = center.x - halfLength;
            xmax = center.x + halfLength;
            ymin = center.y - halfLength;
            ymax = center.y + halfLength;
            zmin = center.z - halfLength;
            zmax = center.z + halfLength;
            ptNWF = Point3<float>(xmin, ymax, zmax);
            ptNEF = Point3<float>(xmax, ymax, zmax);
            ptSWF = Point3<float>(xmin, ymin, zmax);
            ptSEF = Point3<float>(xmax, ymin, zmax);
            ptNWB = Point3<float>(xmin, ymax, zmin);
            ptNEB = Point3<float>(xmax, ymax, zmin);
            ptSWB = Point3<float>(xmin, ymin, zmin);
            ptSEB = Point3<float>(xmax, ymin, zmin);
        }

        const Point3<float> getCenter() { return center; }

        float getHalfLength() { return halfLength; }

        float getHalfLengthSq() { return halfLengthSq; }

        float getXMinbound() { return xmin; }

        float getXMaxbound() { return xmax; }

        float getYMinbound() { return ymin; }

        float getYMaxbound() { return ymax; }

        float getZMinbound() { return zmin; }

        float getZMaxbound() { return zmax; }

        const Point3<float> &getNWF() { return ptNWF; }

        const Point3<float> &getNEF() { return ptNEF; }

        const Point3<float> &getSWF() { return ptSWF; }

        const Point3<float> &getSEF() { return ptSEF; }

        const Point3<float> &getNWB() { return ptNWB; }

        const Point3<float> &getNEB() { return ptNEB; }

        const Point3<float> &getSWB() { return ptSWB; }

        const Point3<float> &getSEB() { return ptSEB; }

        //nearly equal for float comparison
        bool n_equal(float a, float b, float epsilon = 128 * FLT_EPSILON, float abs_th = FLT_MIN) {
            assert(std::numeric_limits<float>::epsilon() <= epsilon);
            assert(epsilon < 1.f);

            if (a == b) return true;
            auto diff = std::abs(a-b);
            auto norm = std::min(std::abs(a + b), std::numeric_limits<float>::max());
            return diff < std::max(abs_th, epsilon * norm);
        }

        //TODO: Rethink if equals is fine here
        bool containsPoint(Point3<float> pt) {
            return (((pt.x > xmin) || n_equal(pt.x, xmin)) &&
                    ((pt.x < xmax) || n_equal(pt.x, xmax)) &&
                    ((pt.y > ymin) || n_equal(pt.y, ymin)) &&
                    ((pt.y < ymax) || n_equal(pt.y, ymax)) &&
                    ((pt.z > zmin) || n_equal(pt.z, zmin)) &&
                    ((pt.z < zmax) || n_equal(pt.z, zmax)));
        }

        bool intersectsAABB(AABB3 aabb) {
            return !((aabb.getXMaxbound() < xmin) ||
                     (aabb.getXMinbound() > xmax) ||
                     (aabb.getYMaxbound() < ymin) ||
                     (aabb.getYMinbound() > ymax) ||
                     (aabb.getZMaxbound() < zmin) ||
                     (aabb.getZMinbound() > zmax));
        }
    };

    class OcTree {
        // binary for x y z --> 1 0 0 means +x -y -z
        const int CHILD_TYPE_SWB = 0b000; // southWestBack
        const int CHILD_TYPE_SWF = 0b001; // southWestFront 
        const int CHILD_TYPE_NWB = 0b010; // northWestBack
        const int CHILD_TYPE_NWF = 0b011; // northWestFront 
        const int CHILD_TYPE_SEB = 0b100; // southEastBack
        const int CHILD_TYPE_SEF = 0b101; // southEastFront
        const int CHILD_TYPE_NEB = 0b110; // northEastBack
        const int CHILD_TYPE_NEF = 0b111; // northEastFront 

        static int s_ID;

        int id;
        int level;
        std::string typeStr;

        // Axis-aligned bounding box stored as a center with half-dimensions
        // to represent the boundaries of this quad tree
        AABB3 boundary;

        tree_param param;    // see strct.h for definition

        // Points in this quad tree node
        std::shared_ptr<Node3> node;
        std::shared_ptr<Node3> closestChildNode; // representative point (clasest to the center)
        std::shared_ptr<IDMP_ros::gp> gp;

        bool leaf;
        bool maxDepthReached;
        bool rootLimitReached;
        bool toRemove = false;

        int32_t numNodes;

        // Children
        OcTree *northWestFront;
        OcTree *northEastFront;
        OcTree *southWestFront;
        OcTree *southEastFront;
        OcTree *northWestBack;
        OcTree *northEastBack;
        OcTree *southWestBack;
        OcTree *southEastBack;

        OcTree *par;

        void Subdivide(); // create four children that fully divide this quad into four quads of equal area
        void SubdivideExcept(int childType);

        void deleteChildren();

        bool InsertToParent(std::shared_ptr<Node3> n);

        OcTree(AABB3 _boundary, tree_param tp, OcTree *const p = 0);

        OcTree(AABB3 _boundary, OcTree *const ch, int child_type,  tree_param tp);

    protected:
        void setParent(OcTree *const p) { par = p; }

        OcTree *const getParent() { return par; }

        bool IsLeaf() { return leaf; } // leaf is true if chlidren are initialized
        bool IsEmpty() { return (node == nullptr); } // empty if the data node is null
        bool IsEmptyLeaf() {
            return (leaf & (node == nullptr)); // true if no data node no child
        }

    public:
        // Methods
        OcTree() : northWestFront(0),
                   northEastFront(0),
                   southWestFront(0),
                   southEastFront(0),
                   northWestBack(0),
                   northEastBack(0),
                   southWestBack(0),
                   southEastBack(0),
                   par(0),
                   maxDepthReached(false),
                   rootLimitReached(false),
                   leaf(true),
                   numNodes(0),
                   node(nullptr),
                   closestChildNode(nullptr),
                   gp(nullptr) {}

        OcTree(Point3<float> c,  tree_param tp);

        ~OcTree() {
            deleteChildren();
        }

        bool IsRoot() {
            if (par)
                return false;
            else
                return true;
        }

        OcTree *const getRoot();

        int getOctant(Point3<float> pt) {
            if(std::abs(boundary.getCenter().x - pt.x) < 0.0001 && std::abs(boundary.getCenter().y - pt.y) < 0.0001 && std::abs(boundary.getCenter().z - pt.z) < 0.0001 ) return 0; //TODO: What happens if its on boundary?
            return (pt.x > boundary.getCenter().x) << 2 | (pt.y > boundary.getCenter().y) << 1 | (pt.z > boundary.getCenter().z);
        }

        bool Insert(std::shared_ptr<Node3> n);

        bool Insert(std::shared_ptr<Node3> n, std::unordered_set<OcTree *> &quads);

        bool IsNotNew(std::shared_ptr<Node3> n);

        bool Update(std::shared_ptr<Node3> n);

        bool Update(std::shared_ptr<Node3> n, std::unordered_set<OcTree *> &quads);

        bool MarkRemoval(std::shared_ptr <Node3> n);

        bool RemoveMarked();


        bool Remove(std::shared_ptr <Node3> n, std::unordered_set<OcTree *> &octs);

        void Update(std::shared_ptr<IDMP_ros::gp> _gp);

        std::shared_ptr<IDMP_ros::gp> const getGP() { return gp; }

        bool Remove(std::shared_ptr<Node3> n);

        void QueryRange(AABB3 range, std::vector<std::shared_ptr<Node3> > &nodes);

        void QueryNonEmptyLevelC(AABB3 range, std::vector<OcTree *> &quads);

        bool getContainingTreeC(IDMP_ros::Point3<float> pt, OcTree* &tree);
        OcTree* findNNC(IDMP_ros::Point3<float> pt);
        void radiusSearch(IDMP_ros::Point3<float> center, float rad, std::vector<OcTree *> &octs, std::vector<float> &sqdst);


        void QueryNonEmptyLevelC(AABB3 range, std::vector<OcTree *> &quads, std::vector<float> &sqdst);
        void QueryNonEmptyDist(IDMP_ros::Point3<float> src, std::vector<float> &sqdst);

        void QueryNonEmptyLevelC(AABB3 range, std::vector<OcTree *> &quads,
                                 std::vector <std::vector<std::shared_ptr < Node3>>

        >& nodes);

        int32_t getNodeCount() { return numNodes; }

        Point3<float> getCenter() { return boundary.getCenter(); }

        float getHalfLength() { return boundary.getHalfLength(); }

        float getMinHalfLength() { return param.min_halfleng; }

        float getXMinbound() { return boundary.getXMinbound(); }

        float getXMaxbound() { return boundary.getXMaxbound(); }

        float getYMinbound() { return boundary.getYMinbound(); }

        float getYMaxbound() { return boundary.getYMaxbound(); }

        float getZMinbound() { return boundary.getZMinbound(); }

        float getZMaxbound() { return boundary.getZMaxbound(); }

        Point3<float> getNWF() { return boundary.getNWF(); }

        Point3<float> getNEF() { return boundary.getNEF(); }

        Point3<float> getSWF() { return boundary.getSWF(); }

        Point3<float> getSEF() { return boundary.getSEF(); }

        Point3<float> getNWB() { return boundary.getNWB(); }

        Point3<float> getNEB() { return boundary.getNEB(); }

        Point3<float> getSWB() { return boundary.getSWB(); }

        Point3<float> getSEB() { return boundary.getSEB(); }

        void getAllChildrenNonEmptyNodes(std::vector<std::shared_ptr<Node3> > &nodes);
        void getAllFrustumNodes(std::vector<std::shared_ptr<Node3> > &nodes, const std::vector<IDMP_ros::Frustum> frustum);


        void updateCount();

        std::vector<OcTree*> getChildren(){
            std::vector<OcTree*> children(8);
            children[0]=northWestFront;
            children[1]=northEastFront;
            children[2]=southWestFront;
            children[3]=southEastFront;
            children[4]=northWestBack;
            children[5]=northEastBack;
            children[6]=southWestBack;
            children[7]=southEastBack;
            return children;
        }

        OcTree* getChild(int child_type) {
            if(child_type == CHILD_TYPE_SWB) return southWestBack;
            else if(child_type == CHILD_TYPE_SWF) return southWestFront;
            else if(child_type == CHILD_TYPE_NWB) return northWestBack;
            else if(child_type == CHILD_TYPE_NWF) return northWestFront;
            else if(child_type == CHILD_TYPE_SEB) return southEastBack;
            else if(child_type == CHILD_TYPE_SEF) return southEastFront;
            else if(child_type == CHILD_TYPE_NEB) return northEastBack;
            else if(child_type == CHILD_TYPE_NEF) return northEastFront;
            return nullptr;
        }

        static inline std::string getChildTypeStr(int ct) {
            static std::string s_child_type_str[] =
                    {"NWF", "NEF", "SWF", "SEF", "NWB", "NEB", "SWB", "SEB"};
            return s_child_type_str[ct];
        }

        void printOcRoot(const std::string &f_path);

        void printPCD(const std::string &f_path);

        //void gatherPCL( pcl::PointCloud<pcl::PointXYZ>& internal );
    };
}

#endif
