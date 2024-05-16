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

class TFconverter:
    def __init__(self):
        self.sub_msg = rospy.Subscriber("/kinect/vrpn_client/estimated_transform", TransformStamped, self.my_callback)

    def my_callback(self, data):
        # self.pub_tf.publish(TFMessage([data]))  # works but no 'nicely readable'

        # clearer
        br = tf2_ros.TransformBroadcaster()

        data.header.frame_id = "base_link"
        br.sendTransform(data)

if __name__ == '__main__':
    rospy.init_node('datasetTFPub')
    sbr = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()
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
    tfb = TFconverter()
    rospy.spin()