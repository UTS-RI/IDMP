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

matplotlib.use("Qt5Agg")

def process_feedback(feedback):
    global pose
    global new_data
    min_dist = 0.005 #move 5mm to get new measurement
    new_pose = ros_numpy.numpify(feedback.pose) # type: ignore
    # if(np.linalg.norm(pose[:3,3]-new_pose[:3,3]) >= min_dist):
    pose = new_pose
    new_data = True

def numpy_to_msg(arr, intensity, frame):
    data = np.zeros(len(arr), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),('Intensity', np.float32)])
    data['x'] = arr[:, 0]
    data['y'] = arr[:, 1]
    data['z'] = arr[:, 2] + intensity
    data['Intensity'] = intensity
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
        m.color.r = 1
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

if __name__=="__main__":
    rospy.init_node("dynQuery")
    # plt.ion()
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("dist_field")
    pcl_pub = rospy.Publisher("distfield", PointCloud2, queue_size=0)
    marker_pub = rospy.Publisher("grad_array", MarkerArray, queue_size=0)
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "dist_field_center"
    int_marker.description = "Movable Distance Field"

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 0.2

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    box_control.markers.append( box_marker )
    int_marker.controls.append( box_control )

    pose = ros_numpy.numpify(box_marker.pose)
    new_data = True

    server.insert(int_marker, process_feedback)
    server.applyChanges()

    # rospy.wait_for_service('query_dist_field')
    query = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)

    doubleSlice = False
    rotate = False

    step = 0.01
    xSize = 3.75
    ySize = 3.2

    queryGrid = np.mgrid[-xSize/2:xSize/2:step, -ySize/2:ySize/2:step, 1:1.1:0.2, 1:1.1:0.2].reshape(4,-1).T
    print("Total query points: ", queryGrid.shape)
    # fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    # plt.show()
    gt_cloud = o3d.io.read_point_cloud("/home/usama/Workspace/ros/utsWs/bags/ladyCow/cow_and_lady_gt.ply") # Read point cloud
    query_cloud = o3d.geometry.PointCloud()
    plt.rcParams['figure.constrained_layout.use'] = True
    while not rospy.is_shutdown():
        if(new_data):
            trans_grid = (pose @ queryGrid.T).T
            #remove homogenous 1
            trans_grid=np.delete(trans_grid,3,axis=1)
            #flatten grid to send to service 
            ser_grid = np.reshape(trans_grid, len(trans_grid)*3)
            #query loggpis
            print("Querying LogGPIS")
            res = query(ser_grid)

            
            grads = np.array(res.gradients).reshape((len(res.gradients)//3,3))
            #new_data = False
            
            #calculate gt dist
            query_cloud.points = o3d.utility.Vector3dVector(trans_grid)
            print("Calculating groundtruth")
            gt_dists = np.array(query_cloud.compute_point_cloud_distance(gt_cloud))

            error = np.subtract(np.abs(np.array(res.distances)), np.abs(gt_dists))
            print("RMSE: ", np.linalg.norm(np.subtract(np.abs(np.array(res.distances)), np.abs(gt_dists)))/np.sqrt(len(error)))
            # pcl_pub.publish(numpy_to_msg(trans_grid, res.distances, "base_link"))
            
            pcl_pub.publish(numpy_to_msg(trans_grid, np.array(res.distances), "base_link"))

            # # plt.cla()
            # c_dist = np.array(res.distances)
            # c_dist = (c_dist - c_dist.min()) / c_dist.ptp()
            # c_dist = plt.cm.hsv(c_dist)
            
            # mArr = MarkerArray()
            # for idx, (pos, grad, col) in enumerate(zip(trans_grid, grads, c_dist)):
            #         mArr.markers.append(create_arrow(0.05,pos,pos+grad*0.3,idx, col))
            # marker_pub.publish(mArr)
            gridSize = int(math.sqrt(len(error)))

            fig, ax = plt.subplots(1,3)
            im1 = ax[0].pcolormesh(trans_grid[:,0].reshape((math.ceil(xSize/step),math.ceil(ySize/step))), trans_grid[:,1].reshape((math.ceil(xSize/step),math.ceil(ySize/step))), np.array(res.distances).reshape((math.ceil(xSize/step),math.ceil(ySize/step))), cmap=plt.cm.gist_rainbow)
            ax[0].set_title("LogGPIS")
            plt.colorbar(im1)
            im2 = ax[1].pcolormesh(trans_grid[:,0].reshape((math.ceil(xSize/step),math.ceil(ySize/step))), trans_grid[:,1].reshape((math.ceil(xSize/step),math.ceil(ySize/step))), gt_dists.reshape((math.ceil(xSize/step),math.ceil(ySize/step))), cmap=plt.cm.gist_rainbow)
            ax[1].set_title("Groundtruth")
            plt.colorbar(im2)
            im3 = ax[2].pcolormesh(trans_grid[:,0].reshape((math.ceil(xSize/step),math.ceil(ySize/step))), trans_grid[:,1].reshape((math.ceil(xSize/step),math.ceil(ySize/step))), error.reshape((math.ceil(xSize/step),math.ceil(ySize/step))), cmap=plt.cm.gist_rainbow)
            ax[2].set_title("Error")
            plt.colorbar(im3)
            # fig.tight_layout()
            # surf = ax.plot_surface(trans_grid[:,0].reshape((gridSize,gridSize)), trans_grid[:,1].reshape((gridSize,gridSize)), np.array(error).reshape((gridSize,gridSize)), cmap=plt.cm.hsv, linewidth=0, antialiased=True)
            plt.show()
            #quiv = ax.quiver(trans_grid[:,0].reshape((50,50,1)), trans_grid[:,1].reshape((50,50,1)),trans_grid[:,2].reshape((50,50,1)),grads[:,0].reshape((50,50,1)), grads[:,1].reshape((50,50,1)),grads[:,2].reshape((50,50,1)), colors=c_dist, length=0.01, normalize=True)
        # plt.pause(0.001)
        rospy.Rate(5).sleep()
        
