#!/usr/bin/env python
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
import sys
print(sys.version)
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from tf.transformations import quaternion_about_axis
class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/rgb/camera_info", CameraInfo, self.camera_info_callback)
        #NOTE: Adjust these two parameters for your printed marker
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.marker_length = 0.100 #width of black square in m
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)
        self.marker_points = np.array([[-self.marker_length / 2, self.marker_length / 2, 0],
                              [self.marker_length / 2, self.marker_length / 2, 0],
                              [self.marker_length / 2, -self.marker_length / 2, 0],
                              [-self.marker_length / 2, -self.marker_length / 2, 0]], dtype=np.float32)

    def camera_info_callback(self, data):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(data.K).reshape(3, 3)
            self.dist_coeffs = np.array(data.D)

    def image_callback(self, data):
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.loginfo("Waiting for camera calibration data...")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            if np.all(ids is not None):
                for i in range(0, len(ids)):
                    _, rvec, tvec = cv2.solvePnP(self.marker_points, corners[i], self.camera_matrix, self.dist_coeffs, None, None, False, cv2.SOLVEPNP_IPPE_SQUARE)
                    # rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.camera_matrix, self.dist_coeffs)
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    # Construct the transformation matrix

                    axis = rvec.flatten() / np.linalg.norm(rvec)
                    angle = np.linalg.norm(rvec)
                    q = quaternion_about_axis(angle, axis)
                    tvec = tvec.flatten()
                    print("TF args: {} {} {} {} {} {} {}".format(tvec[0],tvec[1],tvec[2], q[0], q[1], q[2], q[3]))
            cv2.imshow("Live Image",cv_image)
            cv2.waitKey(10)

        except Exception as e:
            rospy.logerr(e)

def main():
    rospy.init_node('aruco_detector', anonymous=True)
    aruco_detector = ArucoDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
