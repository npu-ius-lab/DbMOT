# How to Use the Experiments Data

Our experimental data includes both simulation and real-world data, all stored as ROS bags.

## Simulation

> **The simulation data includes:**
>
> ROS TF transformations `/tf`, 
>
> YOLO detection results `/yolo/bbx`, 
>
> Drone camera images `/front_cam/camera/image`, 
>
> Drone camera intrinsics `/front_cam/camera/camera_info`,
>
> Drone poses `/ground_truth/state`,
>
> Mini bots odometry information `/tianbot_mini_01/odom`,
>
> and other related topics.

### Camera Intrinsics

$$
\left[\begin{array}{ccc}
319.9988245765257 & 0.000000000000000 & 320.500 \\
0.000000000000000 & 319.9988245765257 & 240.500 \\
0.000000000000000 & 0.000000000000000 & 1.00000
 \end{array}\right]
$$

### Camera Extrinsics

The transformation represents the camera coordinate system relative to the UAV body coordinate system.

​	Translation (x, y, z): $\left[-0.000, -0.229, -0.113\right]^T$

​	Rotation (w, x, y, z): $\left[0.430, 0.561, -0.561, 0.430\right]^T$

### Play the bag

To run the simulation data, use:

```bash
# Terminal One
roscore
# Terminal Two
rosbag play simulation.bag 
```

## Real-world Experiment

>**The real-world experiment data includes:**
>
>Positions of mini robots `/vrpn_client_node/bot/pose`,
>
>Drone pose `/vrpn_client_node/rmtt_02/pose`,
>
>Camera image `/rmtt_02/image_raw`,
>
>and other related topics.

### Camera Intrinsics

$$
\left[\begin{array}{ccc}
919.4281 & 0.000000 & 468.9729 \\
0.000000 &  919.0757 & 365.6025\\
0.000000 & 0.000000 & 1.000000
 \end{array}\right]
$$

### Camera Extrinsics

The transformation represents the camera coordinate system relative to the drone body coordinate system.

​	Translation (x, y, z): $\left[0.0137623, 0.00222272, -0.0298866\right]^T$

​	Rotation (w, x, y, z): $\left[-0.421837,  0.568055, -0.540446,  0.455285\right]^T$

### Play the bag

To run the real-world experiments data, use:

```bash
# Terminal One
roscore
# Terminal Two
rosbag play realworld.bag 
```

## Downloads

[link](https://pan.baidu.com/s/1c6a2QpMuYE-7UxpLfrUu8A?pwd=3f58) 
