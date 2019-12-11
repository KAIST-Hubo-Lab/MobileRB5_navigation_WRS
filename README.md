# path_planning

### Set up
* install ros-kinetic-octomap files (octomap, server, rviz-plugin)
* move the realsense launchfile to realsense directory
* move the rtabmap launchfile to rtabmap directory.

### Running path planning with Rtabmap

* git clone this repository into your catkin workspace
```sh
$ roslaunch realsense2_camera rs_rgbd_hubostanding.launch
$ roslaunch rtabmap_ros rtabmap_hubo.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/aligned_depth_to_color/camera_info rviz:=true rtabmapviz:=false frame_id:=hubo_base_link
$ rosrun mobile_path_planning fake_octomap 
$ rosrun mobile_path_planning mobile_path_planning

```

### Additional notes

* can play off rosbag data. 
* can modify goal pose by publishing new pose




