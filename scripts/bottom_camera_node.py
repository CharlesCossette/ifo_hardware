#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

def gst_pipeline_string(
    capture_width=1920,
    capture_height=1080,
    framerate=30,
    flip_method=2,
):
# TODO: Connect this to a config file
    return (
        "nvarguscamerasrc sensor-mode=2 ! "
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

class BottomCameraNode(object):

    def __init__(self):
        super(BottomCameraNode, self).__init__()

        # Creates a connection to the camera using GStreamer
        framerate = 30 # TODO. remove hardcoding
        self.device = cv2.VideoCapture()
        self.device.open(
                        gst_pipeline_string(framerate=framerate),
                        cv2.CAP_GSTREAMER
                        )    
        # Initalize node and publisher
        rospy.init_node('bottom_camera')  
        self.pub = rospy.Publisher('~image/compressed', CompressedImage, queue_size=1)
              
    def run(self):
        # Publish at dedicated rate
        framerate = 30 # TODO. remove hardcoding
        rate = rospy.Rate(framerate)
        bridge = CvBridge()
        while not rospy.is_shutdown():
            retval, img = self.device.read()
            img = np.uint8(img)
            img_msg = bridge.cv2_to_compressed_imgmsg(img, dst_format='jpeg')
            self.pub.publish(img_msg)
            rate.sleep()

    def shutdown_hook(self):
        self.device.release()

if __name__ == "__main__":
    bottom_camera_node = BottomCameraNode()
    rospy.on_shutdown(bottom_camera_node.shutdown_hook)
    bottom_camera_node.run()
    rospy.spin() 