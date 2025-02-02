# Fisheye & LiDAR Data Synchronization in ROS 2

This repository provides a **ROS 2 node** that synchronizes three fisheye camera image topics and one LiDAR point cloud topic using `ApproximateTimeSynchronizer`. It also includes a script to verify the accuracy of the synchronization.

---

## Usage Instructions

1. **Fix Fisheye Camera Timestamps**  
   - Before synchronizing, ensure the **fisheye camera images** have valid timestamps.
   - If the timestamps are missing or invalid, follow the steps in the [Fix Fisheye Timestamps](https://github.com/Prabuddhi-05/fix_fisheye) repository to correct them.

2. **Play the Modified Bag File**  
   - Once timestamps are fixed, replay the modified `.db3` bag with simulated time:
     ```bash
     ros2 bag play /path/to/modified_bag --clock
     ```

3. **Run the Synchronization Node**  
   - In another terminal, launch the node that synchronizes fisheye images and LiDAR:
     ```bash
     python3 fisheye_lidar_sync.py
     ```

4. **Verify the Synchronized Topics**  
   - Confirm that new synchronized topics are being published:
     ```bash
     ros2 topic list | grep synchronized
     ```
   - (Optional) Check publishing rates:
     ```bash
     ros2 topic hz /synchronized/fisheye_image_SN00012
     ```

5. **(Optional) Verify Synchronization Accuracy**  
   - Use the `sync_checker.py` script (if provided) to confirm timestamps match closely:
     1. Open a new terminal while the modified bag and sync node are running.
     2. Run:
        ```bash
        python3 sync_checker.py
        ```
     3. Observe the time-difference logs. Differences under **10 ms** are generally considered good for most sensor-fusion tasks.

---

## Troubleshooting

- **No messages received?**  
  Ensure the **QoS settings** match the camera/LiDAR publishers (often `BEST_EFFORT`).
- **Missed synchronizations?**  
  Increase the `slop` parameter (e.g., `0.3`) in `ApproximateTimeSynchronizer`.
- **Messages dropping?**  
  Increase the `queue_size` (e.g., `50`) in `ApproximateTimeSynchronizer`.
- **Timestamps still zero?**  
  Re-run the fisheye timestamp-fixing script on the bag file.

---
