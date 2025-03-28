# ros2_lidar_camera_auto_calibrate

Automatically calibrate camera with lidar by performing ICP to lidar pointcloud and depth pointcloud

## Subscribers

`/rgb/camera_info`: Camera Parameters

`/livox/lidar`: Lidar Pointcloud2

`/depth_to_rgb/image_raw`: Depth Image

## Deps

```bash
pip install open3d message-filters opencv-python scipy
```

## Publishers

tf_publisher: will publish frame TF from between `camera_frame` -> `lidar_frame`

PoseStamped_publisher: topic is `camera_to_lidar_RT`, means point_camera * camera_to_lidar_RT = point_lidar