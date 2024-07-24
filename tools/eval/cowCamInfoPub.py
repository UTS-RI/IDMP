#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

def publish_camera_info():
    rospy.init_node('camera_info_publisher', anonymous=True)
    pub = rospy.Publisher('/camera/depth_registered/camera_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

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
    
    while not rospy.is_shutdown():
        pub.publish(camera_info_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass
