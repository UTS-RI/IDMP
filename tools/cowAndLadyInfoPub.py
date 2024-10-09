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
from geometry_msgs.msg import TransformStamped, Transform
from tf2_msgs.msg import TFMessage
import numpy as np
import tf2_ros
import ros_numpy
from sensor_msgs.msg import CameraInfo



def pose_cb(data):
    br = tf2_ros.TransformBroadcaster()
    data.header.frame_id = "base_link"
    br.sendTransform(data)

def create_camInfo():
    camera_info_msg = CameraInfo()
    
    # Standard Kinect v1 calibration parameters
    camera_info_msg.height = 480
    camera_info_msg.width = 640
    camera_info_msg.distortion_model = "plumb_bob"
    
    # Distortion coefficients: [k1, k2, p1, p2, k3]
    camera_info_msg.D = [-0.203, 0.169, 0, 0, 0]
    
    # Intrinsic camera matrix (K)
    camera_info_msg.K = [525.0, 0.0, 319.5,
                         0.0, 525.0, 239.5,
                         0.0, 0.0, 1.0]
    
    # Rectification matrix (R)
    camera_info_msg.R = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]
    
    # Projection matrix (P)
    camera_info_msg.P = [525.0, 0.0, 319.5, 0.0,
                         0.0, 525.0, 239.5, 0.0,
                         0.0, 0.0, 1.0, 0.0]
    return camera_info_msg

if __name__ == '__main__':
    rospy.init_node('cowAndLadyInfoPub')
    # Transform from pose message to tf
    sbr = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()
    # Transform between vicon and kinect
    sys2cam = TransformStamped()
    sys2cam.header.frame_id = "kinect"
    sys2cam.header.stamp = rospy.Time.now()
    mat = np.array([[0.971048, -0.120915, 0.206023, 0.00114049],
                    [0.15701, 0.973037, -0.168959, 0.0450936],
                    [-0.180038, 0.196415, 0.96385, 0.0430765],
                    [0.0, 0.0, 0.0, 1.0]])
    sys2cam.transform = ros_numpy.msgify(Transform, mat)
    sys2cam.child_frame_id = "camera_rgb_optical_frame"
    sbr.sendTransform(sys2cam)
    # Subscriber for pose
    pose_sub = rospy.Subscriber("/kinect/vrpn_client/estimated_transform", TransformStamped, pose_cb)
    # pubish camera info for kinect
    camInfo_pub = rospy.Publisher('/camera/depth_registered/camera_info', CameraInfo, queue_size=10)
    camInfo_msg = create_camInfo()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        camInfo_pub.publish(camInfo_msg)
        rate.sleep()