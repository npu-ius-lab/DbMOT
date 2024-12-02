# Launch the detection node.

Before running DbMOT, you need to launch a detection ROS node. You can use your own YOLOv8 detector or utilize the [yolov8_ros](https://github.com/fbh16/yolov8.git) provided by us. 

## Get the detection node

```bash
cd your_work_space/src
git clone https://github.com/fbh16/yolov8.git
cd .. && catkin_make
source devel/setup.bash
```

## Modify the parameters

Modify the parameters in the launch file to suit your own requirements:

`conf_thresh`: Detection confidence, detections with confidence below this threshold will be filtered out, the default value is 0.5.

`view_image`: Setting to false to disable the visualization window.

`sub`: The name of the ROS topic to subscribe to.

- In the simulation, we need to subscribe to the drone’s camera image topic named `/front_cam/camera/image`; 
- while in the real-world experiment, we need to subscribe the image topic named `/rmtt_02/image_raw`.

`model`: The absolute path to the detection model.

`class_yaml`: The relative path of the model configuration file. Need to modify it according to your own path.

## Launch 

Run the detection node, e.g. :

```bash
roslaunch yolov8_ros v8.launch view_image:=0 conf_thresh:=0.5 sub:=/rmtt_02/image_raw
```
When the terminal displays the following information, it indicates that the detection node is ready:

```bash
Ultralytics YOLOv8.0.166 🚀 Python-3.6.9 torch-1.7.1+cu110 
Model summary (fused): 218 layers, 25840339 parameters, 0 gradients, 78.7 GFLOPs
YOLOv8 is ready
```

## Warning

Please note that **do not launch the detection node again** when using **simulation data**, detection information is included. You only need to subscribe to the detection topic `/yolo/bbx` and `/yolo/img` in the bag.

The usage instructions for simulation and real-world data can be found in [DATA.md](DATA.md).

## Download

The trained detection model and configuration file can be downloaded from [link](https://pan.baidu.com/s/1uY5HgOiBGKkL-9_e8m1K1g?pwd=zsv3).
