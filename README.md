# Fisheye & LiDAR Data Synchronization in ROS 2

This repository contains a **ROS 2 node** that synchronizes 3 fisheye camera image topics and 1 LiDAR point cloud topic using `ApproximateTimeSynchronizer`.

## **Steps to use this repository**

### **Step 1: Fix fisheye camera timestamps**
Before running synchronization, ensure that your **fisheye camera images have valid timestamps**. Use the script to fix missing timestamps in ROS 2 bag files by following the steps here:  
👉 [Fix Fisheye Timestamps](https://github.com/Prabuddhi-05/fix_fisheye)

---

### **Step 2: Play the modified bag file**
Once the timestamps are corrected, play the modified bag file with simulated time enabled:
```bash
ros2 bag play /path/to/modified_bag --clock
```

---

### **Step 3: Run the synchronization node**
Start the synchronization node to align the fisheye camera images and LiDAR point clouds:
```bash
python3 fisheye_lidar_sync.py
```

---

### **Step 4: Verify synchronized topics**
Check if the synchronized topics are being published:
```bash
ros2 topic list | grep synchronized
```
To check the frequency of a topic:
```bash
ros2 topic hz /synchronized/fisheye_image_SN00012
```

---

## **Troubleshooting**
- **No messages received?** Ensure the **QoS settings match** the publisher (`BEST_EFFORT` for sensors).  
- **Missed synchronizations?** Increase `slop` (e.g., `0.3`) in `ApproximateTimeSynchronizer`.  
- **Messages dropping?** Increase `queue_size` (e.g., `50`) in `ApproximateTimeSynchronizer`.    
- **Timestamps still zero?** Re-run the timestamp-fixing script.

---


