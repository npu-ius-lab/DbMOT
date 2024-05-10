# Drone-based MOT Combining CMI and SOT 

> **Drone-based Multi-Object Tracking Combining Camera Motion Information and Single Object Trakcer. (ICGNC 2024)**
>
> For drone-based Multi-Object Tracking (MOT), camera motion makes it challenging to maintain accurate object prediction and tracking on the 2D image plane. In this work, we propose a method that leverages camera motion information to predict object motion in the 3D space and back-project on the 2D image plane, resulting in more robust and stable data association. Furthermore, we integrate a single object tracker into our framework and use object motion prediction to guide its search region for more stable tracking. The evaluation by both simulation and real-world experiments demonstrates the superior performance of our method compared with other popular multi-object trackers. 

## Tracking performance

The process of compensating for the search regions in simulation (above) and real-world experiments (below). The blue box represents the original search region, while the red box represents the adjusted search region. The tracking results are shown with yellow bounding boxes containing the object identity information.
![sregion_simulation](figs/sregion_simulation.png)
![sregion_realworld](figs/sregion_realworld.png)



## Framework

![framework](figs/framework.png)

## How to run

```bash
roscore
```

- Run the rosbag.

```bash
rosbag play xx.bag --pause 
```

- Run the detection node.

```bash
roslaunch yolov8_ros v8.launch view_image:=0 sub:=/rmtt_02/image_raw conf_thresh:=0.5
```

- Run the tracking node.

```bash
roslaunch dbmot_3dmm dbmot_3dmm.launch
```

- Play the bag.
