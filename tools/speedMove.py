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
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Pose
#include <ur_ros_driver/StartJog.h>
#include <ur_ros_driver/JogControl.h>
from idmp_ros.srv import GetDistanceGradient
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

from scipy.spatial.transform import Rotation as R
import time
from tf import transformations

import tf2_ros
from ur_ros_driver.srv import StartJog
from ur_ros_driver.msg import JogControl

def pose_to_numpy(msg):
	return np.dot(
        transformations.translation_matrix(np.array([msg.position.x, msg.position.y, msg.position.z])),
        transformations.quaternion_matrix(np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))
    )

def numpy_to_point(arr):
	if arr.shape[-1] == 4:
		arr = arr[...,:-1] / arr[...,-1]

	if len(arr.shape) == 1:
		return Point(*arr)
	else:
		return np.apply_along_axis(lambda v: Point(*v), axis=-1, arr=arr)

def dtype_to_fields(dtype):
    '''Convert a numpy record datatype into a list of PointFields.
    '''

    # mappings between PointField types and numpy types
    type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                    (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                    (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
    pftype_to_nptype = dict(type_mappings)
    nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

    fields = []
    for field_name in dtype.names:
        np_field_type, field_offset = dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        if np_field_type.subdtype:
            item_dtype, shape = np_field_type.subdtype
            pf.count = np.prod(shape)
            np_field_type = item_dtype
        else:
            pf.count = 1

        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        fields.append(pf)
    return fields

def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    # make it 2d (even if height will be 1)
    cloud_arr = np.atleast_2d(cloud_arr)

    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()
    return cloud_msg

def process_feedback(feedback):
    global pose
    global new_data
    min_dist = 0.005 #move 5mm to get new measurement
    new_pose = pose_to_numpy(feedback.pose)
    # if(np.linalg.norm(pose[:3,3]-new_pose[:3,3]) >= min_dist):
    pose = new_pose
    new_data = True

def numpy_to_msg(arr, intensity, frame):
    data = np.zeros(len(arr), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),('Intensity', np.float32)])
    data['x'] = arr[:, 0]
    data['y'] = arr[:, 1]
    data['z'] = arr[:, 2]
    data['Intensity'] = intensity
    return array_to_pointcloud2(data, stamp=rospy.Time.now(), frame_id=frame) # type:ignore

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
    m.lifetime = rospy.Duration(nsecs=200000000)
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

    m.points = [numpy_to_point(start), numpy_to_point(end)]
    return m

def create_marker():
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "dist_field_center"
    int_marker.description = "Movable Distance Field"

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.05
    box_marker.scale.y = 0.05
    box_marker.scale.z = 0.05
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1
    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    box_control.markers.append( box_marker )

    int_marker.controls.append( box_control )
    int_marker.pose.position = Point(-0.5,0.5,0.35)
    return int_marker

def rotationMatrixToQuaternion1(m):
    #q0 = qw
    t = np.matrix.trace(m)
    q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if(t > 0):
        t = np.sqrt(t + 1)
        q[3] = 0.5 * t
        t = 0.5/t
        q[0] = (m[2,1] - m[1,2]) * t
        q[1] = (m[0,2] - m[2,0]) * t
        q[2] = (m[1,0] - m[0,1]) * t

    else:
        i = 0
        if (m[1,1] > m[0,0]):
            i = 1
        if (m[2,2] > m[i,i]):
            i = 2
        j = (i+1)%3
        k = (j+1)%3

        t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
        q[i] = 0.5 * t
        t = 0.5 / t
        q[3] = (m[k,j] - m[j,k]) * t
        q[j] = (m[j,i] + m[i,j]) * t
        q[k] = (m[k,i] + m[i,k]) * t

    return q

if __name__=="__main__":
    rospy.init_node("dynQuery")
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)
    pose = np.eye(4)
    
    server = InteractiveMarkerServer("dist_field")
    server.insert(create_marker(), process_feedback)
    server.applyChanges()

    marker_pub = rospy.Publisher("grad_array", MarkerArray, queue_size=0)
    query = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)
    toggleJog = rospy.ServiceProxy('/ur_hardware_interface/start_jog', StartJog)
    jogVec = rospy.Publisher("/jog_control", JogControl, queue_size=0)

    isJogging = False
    jogMsg = JogControl()
    jogMsg.feature = 2
    jogMsg.acc = 1.5
    jogMsg.time = 0.01
    speed_scale = 1

    # goals = []
    input("Move marker to Goal 1 and press enter!")
    # goals.append(pose[:3,3])
    goals=pose[:3,3]
    # input("Move marker to Goal 2 and press enter!")
    # goals.append(pose[:3,3])
    # goalNum = 0
    # goal = goals[goalNum]

    while not rospy.is_shutdown():
        try:    
            try:
                transform = buf.lookup_transform("base_link","tcp",rospy.Time()).transform
                tcp_pos = np.array([transform.translation.x,transform.translation.y,transform.translation.z])
                # tcp_pos = tcp[:3,3]
                res = query(tcp_pos)
            except Exception as e:
                print(e)
                continue
            
            goalVec = goals - tcp_pos
            goalDist = np.linalg.norm(goalVec)
            goalVec = 0.2*(goalVec/goalDist)
            if goalDist < 0.03:
                goalVec = 0
            # goalVec/= goalDist
            
            dist = res.distances[0]
            grad = np.array(res.gradients)

            #pcl_pub.publish(numpy_to_msg(trans_grid, dists, "base_link"))
            repVec = np.array([0,0,0])
            distFak = 1-(3*dist)
            if(distFak < 0 and distFak > 1):
                # distFak =  1.5/np.exp(10*dist)#(0.3 - dist)/0.3
                distFak = 0
                # distFak = 1-(4*dist*dist)
            repVec = grad
            
            # repWeight = distFak if distFak < 1 else 1
            resVec = distFak*repVec + (1-distFak)*goalVec
            mArr = MarkerArray()
            mArr.markers.append(create_arrow(0.04,tcp_pos,tcp_pos+repVec*0.5 *distFak,0, (1,0,0,1)))
            mArr.markers.append(create_arrow(0.04,tcp_pos,tcp_pos+goalVec*0.5*(1-distFak),1, (0,0,1,1)))
            mArr.markers.append(create_arrow(0.04,tcp_pos,tcp_pos+resVec*0.5,2, (0,1,0,1)))
            marker_pub.publish(mArr)
            # if(goalDist > 0.05):
            print("Distance to goal: ", goalDist)

            if(not isJogging):
                isJogging = True
                toggleJog(True, 1)
        
            jogMsg.stamp = rospy.Time.now()
            jogTM = np.eye(4)
            jogTM[:3,3] = resVec * 0.3
            jogTM[0,3] *=-1
            jogTM[1,3] *=-1
            # jogTM[2,3] = 0
            quad = rotationMatrixToQuaternion1(jogTM[:3,:3])
            trans = jogTM[:3,3]
            jogMsg.cartesian_goal.translation.x = resVec[0] * speed_scale
            jogMsg.cartesian_goal.translation.y = resVec[1] * speed_scale
            jogMsg.cartesian_goal.translation.z = resVec[2] * speed_scale
            jogMsg.cartesian_goal.rotation.w = quad[0]
            jogMsg.cartesian_goal.rotation.x = quad[1]
            jogMsg.cartesian_goal.rotation.y = quad[2]
            jogMsg.cartesian_goal.rotation.z = quad[3]
            jogVec.publish(jogMsg)

            # else:
            #     if(isJogging):
            #         toggleJog(False, 1)
            #         isJogging = False
                # input("Goal reached! Set new goal and press enter!")
                # goal = pose[:3,3]
                # goalNum = (goalNum+1)%2
        except KeyboardInterrupt:
            print("stopped Program")
            toggleJog(False, 1)
            rospy.sleep(0.5)
            sys.exit()
        rospy.Rate(10).sleep()