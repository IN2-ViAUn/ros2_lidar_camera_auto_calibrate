#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

# 消息类型
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import message_filters

import numpy as np
import open3d as o3d
import cv2
from cv_bridge import CvBridge

# TF 发布
from tf2_ros import TransformBroadcaster, TransformStamped
import builtin_interfaces.msg

class LidarCameraICPNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_icp_node')

        self.bridge = CvBridge()
        self.camera_info_received = False
        self.fx = self.fy = self.cx = self.cy = None
        self.has_recv_both_topic = False
        self.scale = 1000.0
        self.max_correspondence_dist = 0.5
        self.init_transform = np.array([
            [ -0.0074538237382291385, 0.3449980499998037, 0.9385738000324044, -0.0772091589716173],
            [ -0.9998967319124494, 0.008961376478752163, -0.011234818879941735, 0.003656484511527655],
            [ -0.012286906422115232, -0.938560621314138, 0.3448956261709061, 0.14999502597312944],
            [ 0.        ,  0.        ,  0.        ,  1.        ]
        ], dtype=np.float64)

        self.get_logger().info("InitialTransform: " + str(self.init_transform))

        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub_caminfo = self.create_subscription(
            CameraInfo,
            '/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        depth_sub = message_filters.Subscriber(self, Image, '/depth_to_rgb/image_raw')
        lidar_sub = message_filters.Subscriber(self, PointCloud2, '/livox/lidar')


        ats = message_filters.ApproximateTimeSynchronizer(
            [depth_sub, lidar_sub],
            queue_size=10,
            slop=0.02
        )
        ats.registerCallback(self.sync_callback)

        self.get_logger().info("Lidar-Camera ICP Node has started.")

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            # 记录相机内参
            self.fx = msg.k[0]  # K[0] = fx
            self.fy = msg.k[4]  # K[4] = fy
            self.cx = msg.k[2]  # K[2] = cx
            self.cy = msg.k[5]  # K[5] = cy
            self.camera_info_received = True

            self.get_logger().info(f"Got camera info: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def sync_callback(self, depth_msg: Image, lidar_msg: PointCloud2):
        if not self.camera_info_received:
            self.get_logger().warn("Haven't received camera info yet, skip this frame.")
            return

        depth_pcd = self.depth_image_to_pointcloud(depth_msg)

        if depth_pcd is None:
            self.get_logger().warn("Depth image is empty or invalid, skip.")
            return

        lidar_pcd = self.ros_pc2_to_open3d(lidar_msg)
        if lidar_pcd is None:
            self.get_logger().warn("Lidar point cloud is empty, skip.")
            return
        
        
        if self.has_recv_both_topic is True:
            return
        
        self.has_recv_both_topic = True

        max_correspondence_dist = self.max_correspondence_dist
        result_icp = o3d.pipelines.registration.registration_icp(
            source = depth_pcd,
            target = lidar_pcd,
            max_correspondence_distance = max_correspondence_dist,
            init = self.init_transform,
            estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        
        refined_transform = result_icp.transformation

        t_init = self.init_transform[:3, 3]
        t_refined = refined_transform[:3, 3]
        translation_diff = np.linalg.norm(t_refined - t_init)
        R_init = self.init_transform[:3, :3]
        R_refined = refined_transform[:3, :3]
        R_diff = R_refined @ R_init.T
        angle_diff_rad = np.arccos(np.clip((np.trace(R_diff) - 1) / 2.0, -1.0, 1.0))
        angle_diff_deg = np.degrees(angle_diff_rad)

        self.get_logger().info(f"angle_diff_deg: {angle_diff_rad:.5f}, translation_diff: {translation_diff:.5f}")

        if translation_diff > 0.1 or angle_diff_deg > 5.0:
            self.get_logger().warn(
                f"ICP result rejected: translation diff = {translation_diff:.3f} m, "
                f"rotation diff = {angle_diff_deg:.2f}°"
            )
            self.has_recv_both_topic = False
            # recalibrate!!
            return

        self.get_logger().info("ICP done! Refined Transform:\n" + str(refined_transform))


        self.broadcast_transform(refined_transform,
                                 parent_frame="camera_frame",
                                 child_frame="lidar_frame",
                                 stamp=depth_msg.header.stamp)

    def depth_image_to_pointcloud(self, depth_msg: Image):
        """将ROS的 depth image (sensor_msgs/Image) 转成 Open3D 点云。"""
        # 转成 numpy
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Convert depth image failed: {e}")
            return None

        if depth_image is None:
            return None

        height, width = depth_image.shape
        points = []
        for v in range(height):
            for u in range(width):
                d = depth_image[v, u]
                if d == 0:
                    continue
                z = float(d) / self.scale
                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy
                points.append([x, y, z])

        if len(points) == 0:
            return None

        points_np = np.array(points, dtype=np.float32)
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points_np)
        return pc

    def ros_pc2_to_open3d(self, ros_msg: PointCloud2):
        """将 ROS PointCloud2 转为 Open3D 点云 (仅 xyz)。"""
        from sensor_msgs_py import point_cloud2
        field_names = [field.name for field in ros_msg.fields]
        if not all(n in field_names for n in ['x','y','z']):
            self.get_logger().warn("PointCloud2 does not have x, y, z fields.")
            return None

        points_list = []
        for p in point_cloud2.read_points(ros_msg, field_names=('x','y','z'), skip_nans=True):
            points_list.append([p[0], p[1], p[2]])

        if len(points_list) == 0:
            return None

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(np.array(points_list, dtype=np.float32))
        return pc

    def broadcast_transform(self, transform_4x4, parent_frame, child_frame, stamp):
        """将 4x4 齐次变换矩阵以 TF 的形式发布。"""
        t = TransformStamped()
        t.header.stamp = stamp  # 直接使用 depth_msg.header.stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # 旋转 (R) & 平移 (t)
        R = transform_4x4[:3, :3]
        t_xyz = transform_4x4[:3, 3]

        # 从旋转矩阵转四元数
        # 注意：要确保旋转矩阵有效，否则可能数值有问题
        import math
        tr = np.trace(R)
        if tr > 0.0:
            s = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * s
            qx = (R[2,1] - R[1,2]) / s
            qy = (R[0,2] - R[2,0]) / s
            qz = (R[1,0] - R[0,1]) / s
        else:
            # 若 R[0,0] 是最大对角元
            if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
                s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
                qx = 0.25 * s
                qy = (R[0,1] + R[1,0]) / s
                qz = (R[0,2] + R[2,0]) / s
                qw = (R[2,1] - R[1,2]) / s
            elif R[1,1] > R[2,2]:
                s = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
                qx = (R[0,1] + R[1,0]) / s
                qy = 0.25 * s
                qz = (R[1,2] + R[2,1]) / s
                qw = (R[0,2] - R[2,0]) / s
            else:
                s = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
                qx = (R[0,2] + R[2,0]) / s
                qy = (R[1,2] + R[2,1]) / s
                qz = 0.25 * s
                qw = (R[1,0] - R[0,1]) / s

        # 填入变换
        t.transform.translation.x = float(t_xyz[0])
        t.transform.translation.y = float(t_xyz[1])
        t.transform.translation.z = float(t_xyz[2])

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        # 发布 TF
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraICPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
