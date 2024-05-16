'''
    IDMP - Interactive Distance Field Mapping and Planning to Enable Human-Robot Collaboration
    Copyright (C) 2024 Usama Ali

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License v3 as published by
    the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License v3 for more details.

    You should have received a copy of the GNU General Public License v3
    along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.html.

    Authors: Usama Ali <usama.ali@thws.de>
             Adrian Mueller <adrian.mueller@thws.de>
             Lan Wu <Lan.Wu-2@uts.edu.au>
'''

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from idmp_ros.srv import GetDistanceGradient
from sensor_msgs.msg import PointCloud2

import numpy as np
import ros_numpy

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib

from scipy.spatial.transform import Rotation as R
import time
import math

import open3d as o3d
from operator import itemgetter


path = "/mnt/14C0B388C0B36E9C/UTS/ROS/ws_moveit_in_chomp/paper/data/"

def numpy_to_msg(arr, frame):
    data = np.zeros(len(arr), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),('Intensity', np.float32)])
    data['x'] = arr[:, 0]
    data['y'] = arr[:, 1]
    data['z'] = arr[:, 2]
    data['Intensity'] = arr[:, 3]
    return ros_numpy.msgify(PointCloud2, data, stamp=rospy.Time.now(), frame_id=frame) # type:ignore

def create_arrow(scale, start, end, idnum, color=None):
    # make a visualization marker array for the occupancy grid
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = 'base_link'
    m.header.stamp = rospy.Time.now()
    m.id = idnum
    m.type = Marker.ARROW
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1
    m.scale.x = scale*0.5
    m.scale.y = scale
    m.scale.z = 0
    if color is None:
        m.color.r = 255
        m.color.g = 0
        m.color.b = 0
        m.color.a = 1
    else:
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = color[3]

    m.points = [ros_numpy.msgify(Point, start), ros_numpy.msgify(Point, end)]
    return m

def getVoxelCenters(voxSize, paddingStep, xMin, xMax, yMin, yMax, zMin, zMax):
    global xMinBound 
    global xMaxBound 
    global yMinBound 
    global yMaxBound 
    global zMinBound 
    global zMaxBound 
    xMinBound = round(int(xMin/voxSize) * voxSize + voxSize/2,4) #norm range to voxels and then shift by halflenght
    xMaxBound = round(int(xMax/voxSize) * voxSize + voxSize/2,4) 
    yMinBound = round(int(yMin/voxSize) * voxSize + voxSize/2,4)
    yMaxBound = round(int(yMax/voxSize) * voxSize + voxSize/2,4)
    zMinBound = round(int(zMin/voxSize) * voxSize + voxSize/2,4)
    zMaxBound = round(int(zMax/voxSize) * voxSize + voxSize/2,4)
    return np.mgrid[xMinBound-paddingStep:xMaxBound+paddingStep:voxSize, yMinBound-paddingStep:yMaxBound+paddingStep:voxSize, zMinBound-paddingStep:zMaxBound+paddingStep:voxSize]#.reshape(3,-1).T

def create_arrows(grad_slice,color=None):
    mArr = MarkerArray()

    length = 0.1
    for idx, (pos, grad) in enumerate(zip(grad_slice[:,:3], grad_slice[:,4:])):
        # print(pos,grad)
        mArr.markers.append(create_arrow(0.01,pos,pos+grad*length,idx,color))

    return mArr

if __name__=="__main__":
    rospy.init_node("dynQuery")
    pcl_pub = rospy.Publisher("distfield", PointCloud2, queue_size=0)
    gt_pub = rospy.Publisher("gt", PointCloud2, queue_size=0)
    marker_pub = rospy.Publisher("grad_array", MarkerArray, queue_size=0)
    marker_pub_gt = rospy.Publisher("grad_gt", MarkerArray, queue_size=0)
    query = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)

    voxelsize = 0.25

    padding = 3
    paddingStep = padding*voxelsize
    #create grid with padding
    padQueryGrid = getVoxelCenters(voxelsize,paddingStep,-2,2,-2,0.8,0.2,1.5)
    #padQueryGrid = np.mgrid[x_min-paddingStep:x_max+paddingStep:step, y_min-paddingStep:y_max+paddingStep:step, z_min-paddingStep:z_max+paddingStep:step]
    padQueryGridShape = padQueryGrid.shape[1:]
    print(padQueryGridShape)
    #reshape grid to [[x,y,z],[x,y,z],...]
    padQueryGrid = padQueryGrid.reshape(3,-1).T

    gt_cloud = o3d.io.read_point_cloud("/mnt/14C0B388C0B36E9C/UTS/ROS/ws_voxfield/cow_lady/cow_and_lady_gt.ply") # Read point cloud
    query_cloud = o3d.geometry.PointCloud()
    
    #calculate gt dist
    query_cloud.points = o3d.utility.Vector3dVector(padQueryGrid)
    print("Calculating groundtruth")
    gt_dists = np.array(query_cloud.compute_point_cloud_distance(gt_cloud))
    
    dist_3d = gt_dists.reshape(padQueryGridShape[0],padQueryGridShape[1], padQueryGridShape[2])
    
    grads = np.gradient(dist_3d)
    gt_grads = np.hstack((grads[0].reshape(-1,1), grads[1].reshape(-1,1),grads[2].reshape(-1,1)))
    # norm gradient
    gt_grads/= (np.sqrt(np.sum(gt_grads**2, axis=1))).reshape(-1,1)
    
    comb_res = np.hstack((padQueryGrid,gt_dists.reshape(-1,1),gt_grads))
    
    noPadMask = np.logical_and(np.round(padQueryGrid[:,0],4)>=xMinBound,
                np.logical_and(np.round(padQueryGrid[:,0],4)<xMaxBound,
                np.logical_and(np.round(padQueryGrid[:,1],4)>=yMinBound,
                np.logical_and(np.round(padQueryGrid[:,1],4)<yMaxBound,
                np.logical_and(np.round(padQueryGrid[:,2],4)>=zMinBound,
                               np.round(padQueryGrid[:,2],4)<zMaxBound)))))
    
    comb_res = comb_res[noPadMask]
    print(comb_res.shape)

    # save to csv
    print("Save to csv")
    lineDump = []
    lineDump.append("x,y,z,d,dx,dy,dz\n")
    for idx,p in enumerate(comb_res):
        lineDump.append(str(round(p[0],4)) + "," + str(round(p[1],4)) + "," + str(round(p[2],4)) + "," + str(p[3]) + "," + str(p[4]) + "," + str(p[5]) + "," + str(p[6]) +"\n")
    
    if(voxelsize>=0.1):
        name = str(int(voxelsize*1000))
    else:
        name = "0"+str(int(voxelsize*1000))

    file = open("/mnt/14C0B388C0B36E9C/UTS/ROS/ws_moveit_in_chomp/src/IDMP_evaluation/cow_lady/data_2/GT_"+name+".csv", "w")
    file.writelines(lineDump)
    file.close()

    print("Done")
    exit()

    grad_slice = comb_res[comb_res[:,2]==zMinBound+voxelsize*10]

    pcl_pub.publish(numpy_to_msg(grad_slice[:,:4], "base_link"))
    mArr_gt = create_arrows(grad_slice)
    marker_pub_gt.publish(mArr_gt)
    

    # print("IDMP")
    # query_slice = grad_slice[:,0:3]
    # ser_grid = np.reshape(query_slice, len(query_slice)*3)
    # res = query(ser_grid)
    # grads = np.array(res.gradients).reshape((len(res.gradients)//3,3))
    # result = np.hstack((query_slice,grads))
    # color = [0,1,0,1]
    # mArr = create_arrows(result,color)
    # marker_pub.publish(mArr)

    # rospy.Rate(1).sleep()