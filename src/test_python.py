#!/usr/bin/python3

import sys
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError


class TestNode():
    def __init__(self):
        rospy.loginfo("TEST NODE is running...")
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/kitti/camera_color_right/image_raw", Image, self.rgb_callback)
        self.img_detected_pub = rospy.Publisher("/TEST_NODE_IMAGE", Image, queue_size=100)

    def rgb_callback(self, data):
        rospy.loginfo("BEFORE CONVERSION")
        try:
            truth_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
        rospy.loginfo("MIDDLE...")
        try:
            img_msg = self.bridge.cv2_to_imgmsg(truth_img, 'bgr8')
        except CvBridgeError as e:
            print(e)
        rospy.loginfo("AFTER CONVERSION")

        self.img_detected_pub.publish(img_msg)
        
if __name__ == "__main__":
    rospy.init_node('TEST_NODE', anonymous=True)
    tnode = TestNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("...shutting down")
