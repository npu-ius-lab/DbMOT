# Launch the tracking node.

> [trackers/sort](trackers/sort) is an implementation of the ROS node for the [SORT](https://github.com/abewley/sort) algorithm.
>
> [trackers/bytetrack](trackers/bytetrack) is an implementation of the ROS node for the [ByteTrack](https://github.com/ifzhang/ByteTrack) algorithm.
>
> [trackers/ucmctrack](trackers/ucmctrack) and [trackers/ucmctrack_sim](trackers/ucmctrack_sim) are implementations of the ROS node for the [UCMCTrack](https://github.com/corfyi/UCMCTrack.git) algorithm.
>
> [trackers/dbmot_2dmm](trackers/dbmot_2dmm) is the ROS node of our 2D motion model method.
>
> [trackers/dbmot_3dmm](trackers/dbmot_3dmm) and [trackers/dbmot_3dmm_sim](trackers/dbmot_3dmm_sim) are ROS nodes of our 3D motion model methods.



==After starting the experimental data and detection node sequentially, the tracking node can be started.==

```bash
# the first termina
roscore
```

```bash
# the second terminal
rosbag play realworld.bag --pause
```

```bash
# the third terminal
cd your_work_space
source devel/setup.bash
roslaunch yolov8_ros v8.launch view_image:=0 conf_thresh:=0.5 sub:=/rmtt_02/image_raw
```

To run the tracking node:

```bash
# the fourth terminal
cd your_workspace
source devel/setup.bash
roslaunch <package_name> <launch_file_name>
# e.g.
roslaunch dbmot_3dmm dbmot_3dmm.launch
```

