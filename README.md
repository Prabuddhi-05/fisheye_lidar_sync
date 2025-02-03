# Fisheye & LiDAR Data Synchronization in ROS 2

This repository provides a **ROS 2 node** that synchronizes three fisheye camera image topics and one LiDAR point cloud topic using `ApproximateTimeSynchronizer`. It also includes a script to verify the accuracy of the synchronization.

---

## Usage Instructions

1. **Fix fisheye camera timestamps**  
   - Before synchronizing, ensure the **fisheye camera images** have valid timestamps.
   - If the timestamps are missing, follow the steps in the [Fix Fisheye Timestamps](https://github.com/Prabuddhi-05/fix_fisheye) repository to correct them.

2. **Play the modified bag file**  
   - Once timestamps are fixed, replay the modified ROS 2 bag with simulated time:
     ```bash
     ros2 bag play /path/to/modified_bag --clock
     ```

3. **Run the synchronization node**  
   - In another terminal, launch the node that synchronizes fisheye camera images and LiDAR:
     ```bash
     python3 fisheye_lidar_sync.py
     ```

4. **(Optional) Verify the synchronized topics**  
   - Confirm that new synchronized topics are being published:
     ```bash
     ros2 topic list | grep synchronized
     ```
5. **(Optional) Verify synchronization accuracy**  
   - Use the `sync_checker.py` script to confirm timestamps match closely:
     1. Open a new terminal while the modified bag and sync node are running.
     2. Run:
        ```bash
        python3 sync_checker.py
        ```
     3. Observe the time-difference logs.
     
     **NOTE**
    - All of the results observed so far show differences under 10 ms, and we will continue to refine our setup to potentially lower these differences even further.

---

## Troubleshooting

- **No messages received?**  
  Ensure the **QoS settings** match the camera/LiDAR publishers (often `BEST_EFFORT`).
- **Missed synchronizations?**  
  Increase the `slop` parameter (e.g., `0.25`) in `ApproximateTimeSynchronizer`.
- **Messages dropping?**  
  Increase the `queue_size` (e.g., `50`) in `ApproximateTimeSynchronizer`.
- **Timestamps still zero?**  
  Re-run the fisheye timestamp-fixing script on the bag file.

---
