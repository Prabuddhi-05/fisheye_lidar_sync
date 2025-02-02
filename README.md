# Fisheye & LiDAR Data Synchronization (ROS 2)

This repository contains a **ROS 2 node** that synchronizes three fisheye camera image topics and one LiDAR point cloud topic using `ApproximateTimeSynchronizer`.

## **Steps to Use This Repository**

### **Step 1: Fix Fisheye Timestamps**
Before running synchronization, ensure that your **fisheye images have valid timestamps**. Use the script to fix missing or invalid timestamps in ROS 2 bag files by following the steps here:  
👉 [Fix Fisheye Timestamps](https://github.com/Prabuddhi-05/fix_fisheye)

---

### **Step 2: Play the Fixed Bag File**
Once the timestamps are corrected, play the modified bag file with simulated time enabled:
```bash
ros2 bag play /path/to/fixed_bag --clock
```

---

### **Step 3: Run the Synchronization Node**
Start the synchronization node to align the fisheye images and LiDAR point clouds:
```bash
python3 fisheye_lidar_sync.py
```

---

### **Step 4: Verify Synchronized Topics**
Check if the synchronized topics are being published:
```bash
ros2 topic list | grep synchronized
```
To check the frequency of a topic:
```bash
ros2 topic hz /synchronized/fisheye_image_SN00012
```

---

### **Step 5: Visualize Data**
To visualize the synchronized fisheye images:
```bash
rqt_image_view
```
To visualize LiDAR point clouds:
```bash
rviz2
```
Ensure that all topics are correctly aligned in time.

---

## **Troubleshooting**
- **No messages received?** Ensure the **QoS settings match** the publisher (`BEST_EFFORT` for cameras).  
- **Missed synchronizations?** Increase `slop` (e.g., `0.3`) in `ApproximateTimeSynchronizer`.  
- **Messages dropping?** Increase `queue_size` (e.g., `50`).  
- **Timestamps still zero?** Re-run the timestamp-fixing script.

---

## **License**
This project is licensed under the [MIT License](LICENSE).

---

## **Author & Contributions**
- **Author**: [Prabuddhi-05](https://github.com/Prabuddhi-05)
- Contributions are welcome! Please open an issue or a pull request if you would like to improve the project.


