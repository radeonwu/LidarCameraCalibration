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
https://ai4sig.org/2018/07/docker-for-autoware/
https://github.com/CPFL/Autoware-Manuals/blob/master/en/Autoware_UsersManual_v1.1.md#runtime-manager-launching


## Test approach of Autoware [4] with docker
Short summary:
- The code can be fully run, and both camera intrinsic and camera-LIDAR extrinsic can be calculated
- Verification of calibration result is still pending

#### Install autoware via docker
Assuming docker and nvidia-docker runtime have been installed \
Following steps as below \
https://gitlab.com/autowarefoundation/autoware.ai/autoware/wikis/Generic-x86-Docker#run-an-autoware-docker-image

#### Run autoware
The code below uses example data set for illustration purpose.

##### in 1st terminal
```
cd docker/generic
./build.sh
roscore
```

##### in 2nd terminal
```
cd docker/generic
./build.sh
cd ~/shared_dir/
rosbag play -l new_rectified.bag
```
##### in 3rd terminal
```
cd docker/generic
./build.sh
source /opt/ros/kinetic/setup.bash
rosrun rviz rviz
```

##### in 4th terminal
```
cd docker/generic
./build.sh
rosrun autoware_camera_lidar_calibrator cameracalibrator.py --square 0.05 --size 7x5 image:=/sensors/camera/image_color 
# camera intrinsic; the corners will be selected automatically 
roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:=/home/autoware/20190711_0256_autoware_camera_calibration.yaml image_src:=/sensors/camera/image_color 
# camera-lidar extrinsic; pick up corners in image and point cloud alternatively
```

## Test approach of Reference [1] with docker
Short summary: the full code can be run following all the steps, yet the testing result of this approach is not stable - the calibration matrix value vibrates a lot everytime after a new image/pointcloud pair is newly registed. 

create a new network foo using the network command
```
docker network create foo
```
Give docker the rights to access the X-Server, by running below in terminal
```
xhost +local:docker
```

##### in 1st terminal
```
nvidia-docker run \
              -it --rm \
              --net foo \
              --name master \
              ros:kinetic-perception-xenial \
              roscore
```

##### in 2nd terminal

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

##### in 3rd terminal
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
