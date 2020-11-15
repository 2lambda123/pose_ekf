# pose_ekf
Extented Kalman Filter for 6D pose estimation using gps, imu, magnetometer.


# 1. State for kalman filter

[p v q ba bw]

error state 
[dp dv d_theta dba, dbw]

# 2. inertial frame: NED

# 3. running
## 3.1 running with simulator
    cd catkin_ws/src
    git clone git@github.com:libing64/pose_ekf.git
    cd ..
    catkin_make -DCATKIN_WHITELIST_PACKAGES="pose_ekf"
    roslaunch pose_ekf pose_ekf_simulator.launch

# 3.2 rosbag for test (TODO)
