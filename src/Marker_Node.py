#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Polygon, Point 
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image, CameraInfo, PointCloud2


class MarkerNode():
    def __init__(self):
        rospy.loginfo('Marker Node running...')
        self.markers_count = 0
        self.MARKERS_MAX = 15
        self.marker_array = MarkerArray()

        self.pcl = None

        self.loc_sub = rospy.Subscriber('ROS_3D_BBox/location_array', Polygon, self.construct_marker)
        self.pcl_sub = rospy.Subscriber('/kitti/velo/pointcloud', PointCloud2, self.pcl_callback)
        self.marker_array_pub = rospy.Publisher('ROS_3D_BBox/marker_publisher', MarkerArray, queue_size=100)
        self.pcl_pub = rospy.Publisher('ROS_3D_BBox/sync_pcl', PointCloud2, queue_size=100)

    def pcl_callback(self, pcl_data):
        self.pcl = pcl_data

    def construct_marker(self, points_coordinates):
        if(len(points_coordinates.points) != 0):
            for point_pos in points_coordinates.points:
                marker = Marker()
                marker.header.frame_id = 'velo_link'
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                marker.scale.x = 1
                marker.scale.y = 1
                marker.scale.z = 1

                marker.color.a = 0.5
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0

                marker.pose.position.x = point_pos.z #z
                marker.pose.position.y = -point_pos.x #x
                marker.pose.position.z = -point_pos.y #y

                if self.markers_count > self.MARKERS_MAX:
                    self.marker_array.markers.pop(0)

                self.marker_array.markers.append(marker)

                id = 0
                for m in self.marker_array.markers:
                    m.id = id
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