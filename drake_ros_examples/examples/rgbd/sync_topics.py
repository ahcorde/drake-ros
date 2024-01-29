from time import sleep

import rclpy
from rclpy.node import Node

from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

class RGBDSynchonizer(Node):

    def __init__(self):
        super().__init__('rgbd_synchonizer')

        self.camera_color_sub = Subscriber(self, Image, "/color/image_raw", qos_profile=qos_profile_sensor_data)
        self.camera_depth_sub = Subscriber(self, Image, "/depth/image_raw", qos_profile=qos_profile_sensor_data)
        self.points_sub = Subscriber(self, PointCloud2, "/points", qos_profile=qos_profile_sensor_data)
        self.camera_color_info_sub = Subscriber(self, CameraInfo, "/color/camera_info", qos_profile=qos_profile_sensor_data)
        self.camera_depth_info_sub = Subscriber(self, CameraInfo, "/depth/camera_info", qos_profile=qos_profile_sensor_data)

        # self.camera_color_pub = self.create_publisher(Image, '/color_sync/image_raw', qos_profile_sensor_data)
        # self.camera_color_camera_info_pub = self.create_publisher(CameraInfo, '/color_sync/camera_info', qos_profile_sensor_data)
        # self.camera_depth_pub = self.create_publisher(Image, '/depth_sync/image_raw', qos_profile_sensor_data)
        # self.camera_depth_camera_info_pub = self.create_publisher(CameraInfo, '/depth_sync/camera_info', qos_profile_sensor_data)
        # self.points_pub = self.create_publisher(PointCloud2, '/points_sync', qos_profile_sensor_data)
        # self.tss = TimeSynchronizer(
        #     [self.camera_color_sub,
        #      self.camera_depth_sub,
        #      self.points_sub,
        #      self.camera_color_info_sub,
        #      self.camera_depth_info_sub],
        #      20)
        # self.tss.registerCallback(self.gotimage)

        self.ts = ApproximateTimeSynchronizer(
            [self.camera_color_sub,
             self.camera_depth_sub,
             self.points_sub,
             self.camera_color_info_sub,
             self.camera_depth_info_sub],
            20,
            0.01,  # defines the delay (in seconds) with which messages can be synchronized
        )
        self.ts.registerCallback(self.callback)


    def gotimage(self, image, depth_image, points, camera_color_info, camera_depth_info):
        assert image.header.stamp == depth_image.header.stamp
        print("got an Image and CameraInfo")
        self.points_pub.publish(points)
        self.camera_color_pub.publish(image)
        self.camera_depth_pub.publish(depth_image)
        self.camera_color_camera_info_pub.publish(camera_color_info)
        self.camera_depth_camera_info_pub.publish(camera_depth_info)
        print(image.header.stamp)
        print(depth_image.header.stamp)
        print(points.header.stamp)
        print(camera_color_info.header.stamp)
        print(camera_depth_info.header.stamp)
    # camera_depth_pub.publish(depth_image)
    # points_pub.publish(points)

    def callback(self, image, depth_image, points, camera_color_info, camera_depth_info):
        print("got an aprox Image and CameraInfo")
        print(image.header.stamp)
        print(depth_image.header.stamp)
        print(points.header.stamp)
        print(camera_color_info.header.stamp)
        print(camera_depth_info.header.stamp)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = RGBDSynchonizer()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

main()
