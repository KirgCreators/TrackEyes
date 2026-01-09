
## 📌 Requirements

* ROS Noetic
* Arduino with compatible firmware
* Python 3
* Apache 2
* MariaDB
* Flask


# Terminal 1
roscore

# Terminal 2
roslaunch rplidar_ros rplidar_a1.launch

# Terminal 3
cd ~/trackeye_backend
python3 server.py

# Terminal 4
cd ~/trackeye_backend
python3 Lidar8.py
