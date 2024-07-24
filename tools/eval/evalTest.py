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

from idmp_ros.srv import GetDistanceGradient
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib
import math
import numpy as np
import open3d as o3d
import os
import rospy
import sys

matplotlib.use("Qt5Agg")

def calculate_cosine_error(actual_gradients, predicted_gradients):
    cosine_errors = []
    for actual, predicted in zip(actual_gradients, predicted_gradients):
            norm = (np.linalg.norm(actual)*np.linalg.norm(predicted))
            if(norm != 0):
                cosine_errors.append(np.dot(actual,predicted)/norm)
    return np.mean(cosine_errors)

def getVoxelCenters(voxSize, xMin, xMax, yMin, yMax, zMin, zMax):
    xMinBound = round(int(xMin/voxSize) * voxSize + voxSize/2,4) #norm range to voxels and then shift by halflenght
    xMaxBound = round(int(xMax/voxSize) * voxSize + voxSize/2 - voxSize/4,4) 
    yMinBound = round(int(yMin/voxSize) * voxSize + voxSize/2,4)
    yMaxBound = round(int(yMax/voxSize) * voxSize + voxSize/2 - voxSize/4,4)
    zMinBound = round(int(zMin/voxSize) * voxSize + voxSize/2,4)
    zMaxBound = round(int(zMax/voxSize) * voxSize + voxSize/2 - voxSize/4,4)
    return np.mgrid[xMinBound:xMaxBound:voxSize, yMinBound:yMaxBound:voxSize, zMinBound:zMaxBound:voxSize]

if __name__=="__main__":
    if(len(sys.argv) < 3): #storePath, description
        exit("Wrong parameters!")
    if(not os.path.isdir(sys.argv[1])):
        print(sys.argv[1])
        exit("Wrong directory!")
    # rospy.init_node("evalTest")
    query = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)

    
    step = 0.01
    xSize = 3.75
    ySize = 3.2

    queryGrid = getVoxelCenters(0.05, -2, 2, -2, 0.8 , 0.2 , 1.5)
    queryGridShape = queryGrid.shape[1:]
    queryGrid = queryGrid.reshape(3,-1).T
    gt_cloud = o3d.io.read_point_cloud("/home/usama/Workspace/ros/utsWs/bags/ladyCow/cow_and_lady_gt.ply") # Read point cloud
    query_cloud = o3d.geometry.PointCloud()
    plt.rcParams['figure.constrained_layout.use'] = True
            
    #flatten grid to send to service 
    ser_grid = np.reshape(queryGrid, len(queryGrid)*3)
    #query idmp
    res = query(ser_grid)

    grads = np.array(res.gradients).reshape((len(res.gradients)//3,3))
    
    #calculate gt dist
    query_cloud.points = o3d.utility.Vector3dVector(queryGrid)
    gt_dists = np.array(query_cloud.compute_point_cloud_distance(gt_cloud))
    dist_3d = gt_dists.reshape(queryGridShape[0],queryGridShape[1], queryGridShape[2])
    num_grads = np.gradient(dist_3d)
    gt_grads = np.hstack((num_grads[0].reshape(-1,1), num_grads[1].reshape(-1,1),num_grads[2].reshape(-1,1)))
    # norm gradient
    gt_grads/= (np.sqrt(np.sum(gt_grads**2, axis=1))).reshape(-1,1)
    

    error = np.subtract(np.abs(np.array(res.distances)), np.abs(gt_dists))
    rmse = np.linalg.norm(np.subtract(np.abs(np.array(res.distances)), np.abs(gt_dists)))/np.sqrt(len(error))
    error = np.abs(error)

    cosine_error = calculate_cosine_error(grads, gt_grads)
    
    gridSize = int(math.sqrt(len(error)))

    # fig, ax = plt.subplots(1,3)
    # im1 = ax[0].pcolormesh(queryGrid[:,0].reshape((math.ceil(xSize/step),math.ceil(ySize/step))), queryGrid[:,1].reshape((math.ceil(xSize/step),math.ceil(ySize/step))), np.array(res.distances).reshape((math.ceil(xSize/step),math.ceil(ySize/step))), cmap=plt.cm.gist_rainbow)
    # ax[0].set_title("IDMP")
    # plt.colorbar(im1)
    # im2 = ax[1].pcolormesh(queryGrid[:,0].reshape((math.ceil(xSize/step),math.ceil(ySize/step))), queryGrid[:,1].reshape((math.ceil(xSize/step),math.ceil(ySize/step))), gt_dists.reshape((math.ceil(xSize/step),math.ceil(ySize/step))), cmap=plt.cm.gist_rainbow)
    # ax[1].set_title("Groundtruth")
    # plt.colorbar(im2)
    # im3 = ax[2].pcolormesh(queryGrid[:,0].reshape((math.ceil(xSize/step),math.ceil(ySize/step))), queryGrid[:,1].reshape((math.ceil(xSize/step),math.ceil(ySize/step))), error.reshape((math.ceil(xSize/step),math.ceil(ySize/step))), cmap=plt.cm.gist_rainbow)
    # ax[2].set_title("Error")
    # plt.colorbar(im3)
    # fig.suptitle(sys.argv[2]+" RMSE: "+str(round(rmse,4)))
    # fig.set_size_inches(16, 9)
    
    # plt.savefig(sys.argv[1]+"/"+sys.argv[2]+".png", dpi = 300, bbox_inches='tight')
    rmseFile = open(sys.argv[1]+"/report.csv","a")
    rmseFile.write(sys.argv[2]+","+str(rmse)+","+ str(cosine_error))
    rmseFile.close()
    # fig.tight_layout()
    # surf = ax.plot_surface(queryGrid[:,0].reshape((gridSize,gridSize)), queryGrid[:,1].reshape((gridSize,gridSize)), np.array(error).reshape((gridSize,gridSize)), cmap=plt.cm.hsv, linewidth=0, antialiased=True)
    
        # plt.pause(0.001)
        
