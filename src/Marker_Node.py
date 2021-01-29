#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Polygon, Point 
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from ROS_3D_BoundingBox.msg import Location, LocationArray
import tf
from tf.transformations import quaternion_from_euler

import numpy as np



class MarkerNode():
    def __init__(self):
        rospy.loginfo('Marker Node running...')
        self.markers_count = 0
        self.MARKERS_MAX = 10
        self.marker_array = MarkerArray()

        self.tf_pub = tf.TransformBroadcaster()
        self.pcl = None

        self.loc_sub = rospy.Subscriber('ROS_3D_BBox/location_array', LocationArray, self.construct_marker)
        self.pcl_sub = rospy.Subscriber('/kitti/velo/pointcloud', PointCloud2, self.pcl_callback)
        self.marker_array_pub = rospy.Publisher('ROS_3D_BBox/marker_publisher', MarkerArray, queue_size=100)
        self.pcl_pub = rospy.Publisher('ROS_3D_BBox/sync_pcl', PointCloud2, queue_size=100)

    def pcl_callback(self, pcl_data):
        self.pcl = pcl_data

    def construct_marker(self, locations_array):
        print(locations_array)
        if(len(locations_array.locations) != 0):
            for point_pos in locations_array.locations:
                marker = Marker()
                marker.header.frame_id = 'velo_link'
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                # coordinates must be reversed to suit velo_link coordinates in rviz

                marker.scale.x = point_pos.size.z
                marker.scale.y = -point_pos.size.x
                marker.scale.z = -point_pos.size.y

                marker.color.a = 0.5
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                yaw_angle = point_pos.alpha + point_pos.theta_ray + (np.pi/2)
                q = quaternion_from_euler(0,0,yaw_angle)

                marker.pose.orientation.x = q[0]
                marker.pose.orientation.y = q[1]
                marker.pose.orientation.z = q[2]
                marker.pose.orientation.w = q[3]

                # coordinates must be reversed to suit velo_link coordinates in rviz

                marker.pose.position.x = point_pos.point.z #z
                marker.pose.position.y = -point_pos.point.x #x
                marker.pose.position.z = -point_pos.point.y #y

                if self.markers_count > self.MARKERS_MAX:
                    self.marker_array.markers.pop(0)

                self.marker_array.markers.append(marker)
                
                id = 0
                for m in self.marker_array.markers:
                    m.id = id
                    self.tf_pub.sendTransform((m.pose.position.x, m.pose.position.y, m.pose.position.z),
                    (m.pose.orientation.x,
                    m.pose.orientation.y,
                    m.pose.orientation.z,
                    m.pose.orientation.w), rospy.Time.now(), 'objec nr:'+str(m.id), 'velo_link')
                    id += 1

                self.markers_count += 1
            self.marker_array_pub.publish(self.marker_array)
            self.pcl_pub.publish(self.pcl)
        else:
            print('no objects to publish')

    def send_marker(self):
        pass

if __name__ == '__main__':
    rospy.init_node('MarkerNode', anonymous=True)
    mn = MarkerNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('shutting down...')