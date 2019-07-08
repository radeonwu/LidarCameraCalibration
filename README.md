# LidarCameraCalibration
## References

[1] An existing code using ROS and PCL library \
https://github.com/heethesh/lidar_camera_calibration

[2] An another ROS pkg, \
http://wiki.ros.org/but_calibration_camera_velodyne \
https://github.com/robofit/but_velodyne

[3] An Matlab reference code \
https://github.com/YechengLyu/WPI-LiDAR-Camera-Calibration-Toolbox

[4] Autoware code \
https://autoware.readthedocs.io/en/feature-documentation_rtd/DevelopersGuide/PackagesAPI/sensing/autoware_camera_lidar_calibrator.html


## Test approach of Reference [1] with docker

create a new network foo using the network command
```
docker network create foo
```
Give docker the rights to access the X-Server, by running below in terminal
```
xhost +local:docker
```

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
              ros:kinetic-perception-xenial

cd $ROS_PACKAGE_PATH/catkin_ws/
catkin_make
source devel/setup.bash
roslaunch lidar_camera_calibration play_rosbag.launch
```

## on 3rd terminal
enter the active container,
```
docker exec -it talker bash
```

install dependancies inside container,
```
apt-get update &&\
apt-get install -y ros-kinetic-ros-numpy \
                ros-kinetic-tf2-sensor-msgs \
                python-tk \
                python-pip &&\
pip install --upgrade pip &&\
python -m pip install -U matplotlib // this will install version 2.x

source /opt/ros/kinetic/share/catkin_ws/devel/setup.bash
```

run calib
```
rosrun lidar_camera_calibration calibrate_camera_lidar.py --calibrate
```
### ROS Docker reference
[1] https://hub.docker.com/_/ros
