# LidarCameraCalibration

An existing code using ROS and PCL library \
https://github.com/heethesh/lidar_camera_calibration

An another ROS pkg, \
http://wiki.ros.org/but_calibration_camera_velodyne \
https://github.com/robofit/but_velodyne

An Matlab reference code \
https://github.com/YechengLyu/WPI-LiDAR-Camera-Calibration-Toolbox

## Test with docker
### on 1st terminal
```
nvidia-docker run \
              -it --rm \
              --net foo \
              --name master \
              ros:kinetic-perception-xenial \
              roscore
```

### on 2nd terminal

```
nvidia-docker run \
              -it --rm \
              --net foo \
              --name talker \
              -v /tmp/.X11-unix:/tmp/.X11-unix \
              --env DISPLAY=unix$DISPLAY \
              --env ROS_HOSTNAME=talker \
              --env ROS_MASTER_URI=http://master:11311 \
              -v $(pwd)/calibration/:/opt/ros/kinetic/share/catkin_ws/ \
              ros:kinetic-perception-xenial \

cd $ROS_PACKAGE_PATH/catkin_ws/
catkin_make
roslaunch lidar_camera_calibration play_rosbag.launch
```

## on 3rd terminal
```
docker exec -it talker bash
```

### install dependancy inside docker
```
apt-get install ros-kinetic-ros-numpy
apt-get install ros-kinetic-tf2-sensor-msgs
apt-get install python-tk
apt-get install python-pip
pip install --upgrade pip
python -m pip install -U matplotlib ***_this will install version 2.x_***
```

## run calib
```
rosrun auro_calibration calibrate_camera_lidar.py --calibrate
```
