#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

def gstreamer_pipeline_string(
    capture_width=1280,
    capture_height=720,
    framerate=21,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method
        )
    )

device = cv2.VideoCapture()
device.open(gstreamer_pipeline_string(), cv2.CAP_GSTREAMER)
bridge = CvBridge()
rospy.init_node('bottom_camera')
pub = rospy.Publisher('~image/compressed', CompressedImage, queue_size=1)


rate = rospy.Rate(10)

while not rospy.is_shutdown():
    retval, img = device.read()
    img = np.uint8(img)
    img_msg = bridge.cv2_to_compressed_imgmsg(img, dst_format='jpeg')
    pub.publish(img_msg)
    rate.sleep()

device.release()

