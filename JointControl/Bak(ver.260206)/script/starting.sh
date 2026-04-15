#!/bin/bash

# Exit immediately on error
set -e

#/usr/sbin/ethercatctl start





# Export necessary environment variables
export ROS_DOMAIN_ID=1
export ROS2_DISTRO=humble
export CYCLONEDDS_URI=file:///home/kist/cyclonedds/cyclone_dds.xml
export IOX_ROUDI_CONFIG_FILE=/home/kist/cyclonedds/roudi_config.toml


# Source ROS 2 and workspace setup
source /opt/ros/humble/setup.bash
source /home/kist/ros2_ws/install/setup.bash

# Start iox-roudi in background

/opt/ros/humble/bin/iox-roudi --config-file="$IOX_ROUDI_CONFIG_FILE" &
ROUDI_PID=$!

# Wait briefly to ensure iox-roudi is initialized
sleep 2

# Launch MicroStrain IMU node
ros2 launch microstrain_inertial_examples cv7_launch.py &
IMU_PID=$!

# Wait briefly before launching LiDAR
sleep 2

# Launch Ouster LiDAR
ros2 launch ouster_ros sensor.launch.xml \
    viz:=false \
    lidar_mode:=512x20 \
    lidar_frame:=os_link \
    udp_dest:=192.168.1.100 \
    sensor_hostname:=192.168.1.50 \
    use_system_default_qos:=true &

LIDAR_PID=$!

# Wait for all background jobs to finish (keep script alive)
#wait $ROUDI_PID $IMU_PID
wait $ROUDI_PID $IMU_PID $LIDAR_PID
