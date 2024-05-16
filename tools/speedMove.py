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
#include <ur_ros_driver/StartJog.h>
#include <ur_ros_driver/JogControl.h>
from idmp_ros.srv import GetDistanceGradient
from sensor_msgs.msg import PointCloud2
import numpy as np
import ros_numpy

from scipy.spatial.transform import Rotation as R
import time

import tf2_ros
from ur_ros_driver.srv import StartJog
from ur_ros_driver.msg import JogControl

def process_feedback(feedback):
    global pose
    new_pose = ros_numpy.numpify(feedback.pose) # type: ignore
    pose = new_pose

def numpy_to_msg(arr, intensity, frame):
    data = np.zeros(len(arr), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),('Intensity', np.float32)])
    data['x'] = arr[:, 0]
    data['y'] = arr[:, 1]
    data['z'] = arr[:, 2]
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

    m.points = [ros_numpy.msgify(Point, start), ros_numpy.msgify(Point, end)]
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
    return int_marker

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
    jogMsg.acc = 0.2;
    jogMsg.time = 0.01;

    goals = []
    input("Move marker to Goal 1 and press enter!")
    goals.append(pose[:3,3])
    input("Move marker to Goal 2 and press enter!")
    goals.append(pose[:3,3])
    goalNum = 0
    # goal = goals[goalNum]

    while not rospy.is_shutdown():
        try:    
            try:
                tcp = ros_numpy.numpify(buf.lookup_transform("base_link","tcp",rospy.Time()).transform)
                tcp_pos = tcp[:3,3]
                res = query(tcp_pos)
            except Exception as e:
                print(e)
                continue
            
            goalVec = goals[goalNum] - tcp_pos
            goalDist = np.linalg.norm(goalVec)
            goalVec/= goalDist
            
            dist = res.distances[0]
            grad = np.array(res.gradients)

            #pcl_pub.publish(numpy_to_msg(trans_grid, dists, "base_link"))
            repVec = np.array([0,0,0])
            distFak = 0
            if(dist < 0.5):
                distFak =  1.5/np.exp(10*dist)#(0.3 - dist)/0.3
                repVec = grad
            
            repWeight = distFak if distFak < 1 else 1
            resVec = distFak*repVec + (1-repWeight)*goalVec
            mArr = MarkerArray()
            mArr.markers.append(create_arrow(0.04,tcp_pos,tcp_pos+repVec*0.5 *distFak,0, (1,0,0,1)))
            mArr.markers.append(create_arrow(0.04,tcp_pos,tcp_pos+goalVec*0.5*(1-repWeight),1, (0,0,1,1)))
            mArr.markers.append(create_arrow(0.04,tcp_pos,tcp_pos+resVec*0.5,2, (0,1,0,1)))
            marker_pub.publish(mArr)
            if(goalDist > 0.05):
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
                jogMsg.cartesian_goal = ros_numpy.msgify(Transform, jogTM)
                jogVec.publish(jogMsg)
            else:
                if(isJogging):
                    toggleJog(False, 1)
                    isJogging = False
                # input("Goal reached! Set new goal and press enter!")
                # goal = pose[:3,3]
                goalNum = (goalNum+1)%2
        except KeyboardInterrupt:
            print("stopped Program")
            toggleJog(False, 1)
            rospy.sleep(0.5)
            sys.exit()
        rospy.Rate(10).sleep()