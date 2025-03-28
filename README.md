# ros2_lidar_camera_auto_calibrate

Automatically calibrate camera with lidar by performing ICP to lidar pointcloud and depth pointcloud

## Subscribers

`/rgb/camera_info`: Camera Parameters

`/livox/lidar`: Lidar Pointcloud2

`/depth_to_rgb/image_raw`: Depth Image

## Deps

```bash
pip install open3d message-filters opencv-python
```