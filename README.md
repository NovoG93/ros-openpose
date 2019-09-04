# openpose-ros
ROS1 wrapper for CMU's Openpose

![posefacehands](pose_face_hands.gif)

Fork of [jacques-saraydaryan/ros-openpose](https://github.com/jacques-saraydaryan/ros-openpose)

## Implementation

- [X] Broadcasting Ros Message
- [X] Humans Pose Estimation
- [X] Bounding Box Calculation + Publishing
- [ ] 3D Pose Estimation of Keypoints using Depth-Camera
- [ ] Face Landmark
- [ ] Hand Pose Estimation

## Installation 
### Openpose

See [Openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose).
Tested with OpenPose 1.5.0
Openpose requires the path to its model fodler. This can be done by either setting     
`$ export OPENPOSE_HOME=<path/to/openpose>`    
or by defining the path in [config.yaml](openpose_ros_node/config/config.yaml)

### Packages

```
$ sudo apt-get install ros-melodic-image-common ros-melodic-vision-opencv ros-melodic-video-stream-opencv ros-melodic-image-view
```

### Catkin Build 

```
$ cd catkin_ws/src
$ git clone https://github.com/NovoG93/ros-openpose
$ cd .. && catkin build
```

## Subscribed Topics
* **`/input_stream`** [[sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)]: Input RGB image

## Published Topics
* **`/openpose_ros_node/output_stream `** [[sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)]: Output RGB image with overlayed Bounding Box and Skeleton
* **`/openpose_ros_node/pose2d `** [[openpose_ros_msgs/Persons](openpose_ros_msgs/msg/Persons.msg)]: List of detected persons with corresponding keypoints
* **`/openpose_ros_node/bboxes `**[[openpose_ros_msgs/BoundingBoxes](openpose_ros_msgs/msg/BoundingBoxes.msg)]: Bounding box arround each detected person

### Message

BodyParts are stored as indexed   
    Nose = 0  
    Neck = 1  
    RShoulder = 2  
    RElbow = 3  
    RWrist = 4  
    LShoulder = 5  
    LElbow = 6  
    LWrist = 7  
    RHip = 8  
    RKnee = 9  
    RAnkle = 10  
    LHip = 11  
    LKnee = 12  
    LAnkle = 13  
    REye = 14  
    LEye = 15  
    REar = 16  
    LEar = 17  

### Parameter
Openpose parameters are stored in [config.yaml](openpose_ros_node/config/config.yaml)

ROS specific parameters are stored in the ROS parameter server and can be set using rqt_reconfigure


    
## Example

### Launch File Example

```
<launch>
    <rosparam command="load" file="$(find openpose_ros_node)/config/config.yaml"/>

    <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
        <param name="video_device" value="/dev/video0" />
        <param name="image_width" value="640" />
        <param name="image_height" value="480" />
        <param name="pixel_format" value="yuyv" />
        <param name="camera_frame_id" value="usb_cam" />
        <param name="io_method" value="mmap"/>
    </node>

    <node name="openpose_node" pkg="openpose_ros_node" type="openpose_node" output="screen"
    launch-prefix="xterm -hold -e" >
    <remap from="input_stream" to="/usb_cam/image_raw"/>
    </node>

    <node name="image_view" pkg="rqt_image_view" type="rqt_image_view" args="/openpose_node/output_stream"/>
</launch>

```