#!/usr/bin/env python3

import rospy
# import tf
# from tf.transformations import quaternion_from_euler as q
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header, Float64MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, Polygon
from ROS_3D_BoundingBox.msg import Location, LocationArray

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from torch_lib.Dataset import *
from library.Math import *
from library.Plotting import *
from torch_lib import Model, ClassAverages
from yolo.yolo import cv_Yolo

import os
import time

import torch
import torch.nn as nn
from torch.autograd import Variable
from torchvision.models import vgg

import argparse

class MakeBoundingBox():
    def __init__(self):
        rospy.loginfo("pointcloud object detection is running...")

        # frame size
        self.frame_x = 640
        self.frame_y = 480

        self.bridge = CvBridge()

        # cv_image and pcl variables
        self.cv_image = np.zeros([self.frame_x, self.frame_y])
        self.pcl = None

        # transform config
        # self.tf_pub = tf.TransformBroadcaster()

        # load torch
        weights_path = os.path.abspath(os.path.dirname(__file__)) + '/weights'
        model_lst = [x for x in sorted(os.listdir(weights_path)) if x.endswith('.pkl')]
        if len(model_lst) == 0:
            print('No previous model found, please train first!')
            exit()
        else:
            print('Using previous model %s'%model_lst[-1])
            my_vgg = vgg.vgg19_bn(pretrained=True)
            self.model = Model.Model(features=my_vgg.features, bins=2).cuda()
            checkpoint = torch.load(weights_path + '/%s'%model_lst[-1])
            self.model.load_state_dict(checkpoint['model_state_dict'])
            self.model.eval()

        # load yolo
        yolo_path = os.path.abspath(os.path.dirname(__file__)) + '/weights'
        self.yolo = cv_Yolo(yolo_path)

        self.averages = ClassAverages.ClassAverages()

        # TODO: clean up how this is done. flag?
        self.angle_bins = generate_bins(2)
        
        calib_path = os.path.abspath(os.path.dirname(__file__)) + "/" + "camera_cal/"
        self.calib_file = calib_path + "calib_cam_to_cam.txt"

        # subscribers
        self.img_sub = rospy.Subscriber("/kitti/camera_color_right/image_raw", Image, self.rgb_callback)
        #self.pcl_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.pcl_callback)
        # publishers
        self.img_detected_pub = rospy.Publisher("ROS_3D_BBox/img_detected_frame", Image, queue_size=100)
        self.location_pub = rospy.Publisher("ROS_3D_BBox/location_array", LocationArray, queue_size=100)
        self.rate = rospy.Rate(1)

    def plot_regressed_3d_bbox(self, img, cam_to_img, box_2d, dimensions, alpha, theta_ray, img_2d=None):

        # the math! returns X, the corners used for constraint
        location, X = calc_location(dimensions, cam_to_img, box_2d, alpha, theta_ray)

        orient = alpha + theta_ray

        if img_2d is not None:
            plot_2d_box(img_2d, box_2d)

        ret_img = plot_3d_box(img, cam_to_img, orient, dimensions, location) # 3d boxes

        return location, ret_img

    def rgb_callback(self, img_data):
        
        try:
            truth_img = self.bridge.imgmsg_to_cv2(img_data, 'bgr8')
        except CvBridgeError as e:
            print(e)
        
        img = np.copy(truth_img)
        yolo_img = np.copy(truth_img)

        start_time = time.time()
        detections = self.yolo.detect(yolo_img)
        locations = []

        for detection in detections:

            

            if not self.averages.recognized_class(detection.detected_class):
                continue

            # this is throwing when the 2d bbox is invalid
            # TODO: better check
            try:
                detectedObject = DetectedObject(img, detection.detected_class, detection.box_2d, self.calib_file)
            except:
                continue

            theta_ray = detectedObject.theta_ray
            input_img = detectedObject.img
            proj_matrix = detectedObject.proj_matrix
            box_2d = detection.box_2d
            detected_class = detection.detected_class

            input_tensor = torch.zeros([1,3,224,224]).cuda()
            input_tensor[0,:,:,:] = input_img

            [orient, conf, dim] = self.model(input_tensor)
            orient = orient.cpu().data.numpy()[0, :, :]
            conf = conf.cpu().data.numpy()[0, :]
            dim = dim.cpu().data.numpy()[0, :]

            dim += self.averages.get_item(detected_class)

            argmax = np.argmax(conf)
            orient = orient[argmax, :]
            cos = orient[0]
            sin = orient[1]
            alpha = np.arctan2(sin, cos)
            alpha += self.angle_bins[argmax]
            alpha -= np.pi

            location, ret_img = self.plot_regressed_3d_bbox(img, proj_matrix, box_2d, dim, alpha, theta_ray, truth_img)
            loc_msg = Location()
            loc_msg.point.x, loc_msg.point.y, loc_msg.point.z = location[0], location[1], location[2]
            loc_msg.size.x, loc_msg.size.y, loc_msg.size.z =  dim[0], dim[1], dim[2]
            loc_msg.alpha = alpha
            loc_msg.theta_ray = theta_ray
            loc_msg.object_type = detection.detected_class
            
            locations.append(loc_msg)
            
        try:
            img_msg = self.bridge.cv2_to_imgmsg(ret_img, 'bgr8')
        except CvBridgeError as e:
                print(e)
        
        loc_msg_header = Header()
        loc_msg_header.frame_id = 'locations array'
        loc_msg_header.stamp = img_msg.header.stamp
        loc_arr_msg = LocationArray(loc_msg_header, locations)

        self.img_detected_pub.publish(img_msg)
        self.location_pub.publish(loc_arr_msg)
        print("\n")
        print('Got %s poses in %.3f seconds FPS: %.2f'%(len(detections), time.time() - start_time, 1/(time.time() - start_time)))
        print('-------------')
        with open('data.csv', 'a') as f:
            f.write(str(len(detections))+','+str(time.time() - start_time)+','+str(1/(time.time() - start_time))+'\n')
        

        
if __name__ == '__main__':
    rospy.init_node('Bbox_Node', anonymous=True)
    mbb = MakeBoundingBox()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("...shutting down")
