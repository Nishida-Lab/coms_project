# 3D SLAM with COMS and Velodyne

![3d_slam_example](./docs/rviz_screenshot_2015_10_30-00_19_49.png "3d_slam_example")

## How to build

```bash
roscore
```

if you use `bag` files,  

```bash
rosparam use_sim_time true
```

```bash
roslaunch coms_description coms.launch
```

```bash
roslaunch coms_mapping pc2ls.launch
```

```bash
roslaunch coms_mapping hector_2d.launch
```

```bash
roslaunch coms_mapping ndt_scan_matching.launch
```

```bash
roslauch coms_mapping octomap_mapping.launch
```

if you use `bag` files,  

```bash
rosbag play hogehoge.bag -r 0.1 --clock
```

Now, it is difficult to build 3dmap on time only CPU.  

Then, map is build, you can save below command,  

```bash
rosrun octomap_server octomap_saver -f mapfile.ot
```

## How to play 3D map

```bash
mv mapfile.ot <catkin_ws>/src/coms_project/coms_mapping/3dmap/
roslaunch coms_mapping octomap_tracking_server.launch
```
