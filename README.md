# LidarCameraCalibration

**Table of Contents** \
[References](##References) \
[Test approach of Autoware](#Autoware-with-docker) \
[Test approach of existing ROS Code](##Test-approach-of-Reference-[1]-with-docker)

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


## Autoware with docker
Short summary:
- The code can be fully run, and both camera intrinsic and camera-LIDAR extrinsic can be calculated
- Verification of calibration result is still pending

#### Install autoware via docker
Assuming docker and nvidia-docker runtime have been installed \
Following steps as below \
https://gitlab.com/autowarefoundation/autoware.ai/autoware/wikis/Generic-x86-Docker#run-an-autoware-docker-image

#### Run autoware calibration
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
```
camera intrinsic; the corners will be selected automatically 
```
roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:=/home/autoware/20190711_0256_autoware_camera_calibration.yaml image_src:=/sensors/camera/image_color 
```
camera-lidar extrinsic; pick up 4 corners in each image and point cloud pair alternatively
- in the image window, click chessboard corner 
- in rviz, tick "Publish Point", then zoom/rotate/translate to find the corresponding point inside point cloud and select it

until the transformation matrix is generated and the values are stable (in theory, the more image-lidar pairs are ticked, the more accurate for transformation values).

#### Verify the calculated transformation 
by projecting point cloud into image frame

Close the autoware_camera_lidar_calibrator node in Terminal 4, and continue with the following
```
rosrun calibration_publisher calibration_publisher  image_topic_name:=/sensors/camera/image_color
```
in rviz started by Terminal 3, select Panels –> Add New Panel –> ImageViewerPlugin，and then in the ImageViewerPlugin sub window, tick select Image Topic & Point Topics

##### in 5th terminal
```
cd docker/generic
./build.sh
rosrun points2image points2image _points_node:=/sensors/velodyne_points
```
The reason to run calibration_publisher and points2image manually as above is due to when launching them from the autoware runtime manager, options are missing or error occurs.

In case by default calibration_publisher does not use the latest calibration yaml file, run the following
```
rosparam set /calibration_publisher/calibration_file /home/autoware/20190716_083429_autoware_lidar_camera_calibration.yaml
```
In case calibration_publisher does not publish /projection_matrix topic as expected, run the following
```
rosparam set /calibration_publisher/publish_extrinsic_mat true
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
