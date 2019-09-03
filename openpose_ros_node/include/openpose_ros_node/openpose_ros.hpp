#ifndef OPENPOSE_ROS_H
#define OPENPOSE_ROS_H

// ROS //
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "openpose_ros_common.hpp"
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>

#include <openpose_ros_msgs/Persons.h>
#include <openpose_ros_msgs/BodyPartDetection.h>
#include <openpose_ros_msgs/PersonDetection.h>
#include "openpose_ros_srvs/DetectPeoplePoseFromImg.h"
#include <openpose_ros_msgs/BoundingBoxes.h>
#include <openpose_ros_msgs/BoundingBox.h>

// C++ //
#include "iostream"
#include "yaml-cpp/yaml.h"

// OpenPose //
#include "openpose/headers.hpp"
#include <openpose/face/headers.hpp>

                            




class openpose_node{
    public:
        openpose_node(YAML::Node config);
        int init_openpose();

    private:
        ros::NodeHandle n;
        image_transport::ImageTransport img_t;
        image_transport::Subscriber sub;
        image_transport::Publisher publish_result;
        ros::Publisher publish_pose, publish_bbox;
        //ros::ServiceServer pose_srv;

        //---OpenPose Vars---//
        op::Point<int> outputSize, netInputSize, netOutputSize;
        op::PoseModel poseModel;

        std::unique_ptr<op::CvMatToOpInput> cvMatToOpInput;
        std::unique_ptr<op::CvMatToOpOutput> cvMatToOpOutput;
        std::unique_ptr<op::OpOutputToCvMat> OpOutputToCvMat;

        std::unique_ptr<op::ScaleAndSizeExtractor> scaleAndSizeExtractor;

            // Caffe //
            std::unique_ptr<op::PoseExtractorCaffe> poseExtractorCaffe;
            std::unique_ptr<op::PoseCpuRenderer> poseRenderer; //ToDo: Change to GPU


        int cnt = 0;
        // Flags 
        bool init_finished, show_skeleton, show_bbox;
        int logging_level, num_gpu_start, scale_number;
        float flag_render_thrash, flag_alpha_pose, scale_gap;
        bool flag_disable_blending;
        std::string flag_net_resolution, flag_model_pose, flag_output_resolution;
        std::string flag_model_folder;

        // Camera
        std::string from_image_topic, to_image_topic;
        int image_width, image_height;


        void pub_bbox(openpose_ros_msgs::Persons persons, cv::Mat &outputImage);

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        //bool peoplePoseFromImg(openpose_ros_srvs::DetectPeoplePoseFromImg::Request  &req,
        //                openpose_ros_srvs::DetectPeoplePoseFromImg::Response &res);
        openpose_ros_msgs::Persons processImg(cv_bridge::CvImagePtr &cv_ptr);
        
};


#endif 