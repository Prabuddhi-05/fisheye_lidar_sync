# Fisheye Camera & LiDAR Synchronization in ROS 2

This repository contains a **ROS 2 node** that synchronizes three fisheye camera image topics and one LiDAR point cloud topic using `ApproximateTimeSynchronizer`.

## Fixing Fisheye Timestamps (Before Synchronization)

1. Before running synchronization, ensure that your **fisheye images have valid timestamps**.  
A **script** to fix missing/invalid timestamps in fisheye camera images in ROS 2 bag files.
Follow the steps here:  
👉 [Fix Fisheye Timestamps](https://github.com/Prabuddhi-05/fix_fisheye)

---

## Synchronization Node

### **Overview**
This ROS 2 node:
- Subscribes to three fisheye image topics (`/fisheye_image_SN00012`, `/fisheye_image_SN00013`, `/fisheye_image_SN00014`).
- Subscribes to one LiDAR point cloud topic (`/front_lidar/points`).
- Uses `ApproximateTimeSynchronizer` to synchronize them.
- Publishes the synchronized messages on `/synchronized/...` topics.

