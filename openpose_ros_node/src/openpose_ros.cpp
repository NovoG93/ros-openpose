#include <openpose_ros_node/openpose_ros.hpp>
#include "openpose/headers.hpp"
#include <thread>

openpose_node::openpose_node(YAML::Node config):
    init_finished(false),
    n("~"),
    img_t(n)
    {
        n.param<bool>("show_skeletion", show_skeleton, true);
        n.param<bool>("show_bbox", show_bbox, true);
        n.param<int>("image_width", image_width, 640);
        n.param<int>("image_height", image_height, 480);
        n.param<int>("flag_logging_level", logging_level, 3);
        n.param<std::string>("flag_net_resolution", flag_net_resolution, "-1x368" );
        n.param<std::string>("flag_model_folder", flag_model_folder, "/home/georg/git/Programms/openpose");
        n.param<std::string>("flag_model_pose", flag_model_pose, "COCO");
        n.param<float>("flag_render_thrash", flag_render_thrash, 0.05);
        n.param<bool>("flag_disable_blending", flag_disable_blending, false);
        n.param<float>("flag_alpha_pose", flag_alpha_pose, 0.6);
        n.param<int>("num_gpu_start", num_gpu_start, 0);
        n.param<std::string>("flag_output_resolution", flag_output_resolution, "-1x-1");
        n.param<int>("scale_number", scale_number, 1);
        n.param<float>("scale_gap", scale_gap, 0.3);

        sub = img_t.subscribe("/usb_cam/image_raw", 1, &openpose_node::imageCallback, this);
        publish_result = img_t.advertise("output_stream", 1);
        publish_pose = n.advertise<openpose_ros_msgs::Persons>("/openpose/pose2d",1);

        ROS_INFO("Init %s node", ros::this_node::getName().c_str());
    }


void openpose_node::pub_bbox(openpose_ros_msgs::Persons persons)
{
    const int num_persons = persons.persons.size();
    const int num_parts = persons.persons[num_persons-1].body_part.size();
    //std::vector<openpose_ros_msgs::BodyPartDetection>::iterator it_min = 
    //    std::min_element(persons.persons[num_persons-1].body_part.begin(), persons.persons[num_persons-1].body_part.end());
    ROS_WARN_STREAM("Detected "<<num_persons << "person(s)");
}

void openpose_node::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (init_finished)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            return;
        }
        if (cv_ptr->image.empty()) return;
        else 
        {
            openpose_ros_msgs::Persons persons = processImg(cv_ptr);
        }
    } else return;
}

openpose_ros_msgs::Persons openpose_node::processImg(cv_bridge::CvImagePtr &cv_ptr)
{
    // Init //
    openpose_ros_msgs::Persons persons;

    std::vector<float> scaleRations;
    
    // Process
    const op::Point<int> imageSize{cv_ptr->image.cols, cv_ptr->image.rows};
    std::vector<double> scaleInputToNetInputs;
    std::vector<op::Point<int>> netInputSizes;
    double scaleInputToOutput;
    op::Point<int> outputResolution;
    scaleAndSizeExtractor = std::make_unique<op::ScaleAndSizeExtractor>(netInputSize, outputSize, scale_number, scale_gap);
    std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution)
        = scaleAndSizeExtractor->extract(imageSize);
        // Format input image to OpenPose input and output formats
    cvMatToOpInput = std::make_unique<op::CvMatToOpInput>(poseModel);
    const auto netInputArray = cvMatToOpInput->createArray(cv_ptr->image, scaleInputToNetInputs, netInputSizes);
    auto outputArray = cvMatToOpOutput->createArray(cv_ptr->image, scaleInputToOutput, outputResolution);
        // Estimate poseKeypoints
    poseExtractorCaffe->forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
    const auto poseKeypoints = poseExtractorCaffe->getPoseKeypoints();

    persons.rostime = ros::Time::now();
    persons.image_w = outputSize.x;
    persons.image_h = outputSize.y;
    
    const int num_people = poseKeypoints.getSize(0);
    const int num_bodyparts = poseKeypoints.getSize(1);
    ROS_DEBUG_STREAM("Detected "<< num_people << " person(s) in the current image");
    ROS_DEBUG_STREAM("The current number of bodyparts in the image:\n \t"<<num_bodyparts);

    for (size_t person_idx = 0; person_idx < num_people; person_idx++) {
        openpose_ros_msgs::PersonDetection person;
        for (size_t bodypart_idx = 0; bodypart_idx < num_bodyparts; bodypart_idx++) {
            size_t final_idx = 3*(person_idx*num_bodyparts + bodypart_idx);
            openpose_ros_msgs::BodyPartDetection bodypart;
            bodypart.part_id = bodypart_idx;
            bodypart.x = poseKeypoints[final_idx];
            bodypart.y = poseKeypoints[final_idx+1];
            bodypart.confidence = poseKeypoints[final_idx+2];
            person.body_part.push_back(bodypart);
        }
        persons.persons.push_back(person);
    }
    if(show_bbox)
    {
        std::thread t1(&openpose_node::pub_bbox, this, persons);
        t1.join();
    }
    /*!
    Publish image stream with overlayed openpose skeleton
    */
    if(show_skeleton)
    {
        poseRenderer->renderPose(outputArray, poseKeypoints, scaleInputToOutput);
        // Openpose Outputarray to OpenCV Image
        auto outputImage = OpOutputToCvMat->formatToCvMat(outputArray);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();
        publish_result.publish(msg);
    }

    ROS_DEBUG_STREAM_THROTTLE(1,"Skeleton Message = \n" << persons);
    publish_pose.publish(persons);
    return persons;
}


int openpose_node::init_openpose()
{
        if (getenv("OPENPOSE_HOME")){
            flag_model_folder=getenv("OPENPOSE_HOME");
            ROS_WARN("Using OPENPOSE_HOME environment var (%s)", flag_model_folder.c_str());
        }else{
            ROS_WARN("OPENPOSE_HOME environment var not set, ... using path from configuration file (%s)", flag_model_folder.c_str());
        }
        flag_model_folder.append("/models/");

        op::check(0 <= logging_level && logging_level <= 255, "Wrong logging_level value.", __LINE__, __FUNCTION__, __FILE__);
        op::ConfigureLog::setPriorityThreshold((op::Priority)logging_level);

        const auto timerBegin = std::chrono::high_resolution_clock::now();

        // Apply configurations 
        netInputSize = op::flagsToPoint(flag_net_resolution, "-1x368");
        netOutputSize = netInputSize;
        outputSize = op::flagsToPoint(flag_output_resolution, "-1x-1");
        poseModel = op::flagsToPoseModel(flag_model_pose);

        //---Initialize---//
            // Convert between CV and OP 
        cvMatToOpOutput = std::make_unique<op::CvMatToOpOutput>();
        OpOutputToCvMat = std::make_unique<op::OpOutputToCvMat>();
            // Caffe
        const auto poseModel = op::flagsToPoseModel(flag_model_pose);
        poseExtractorCaffe = std::make_unique<op::PoseExtractorCaffe>(poseModel, flag_model_folder, num_gpu_start);
        poseExtractorCaffe->initializationOnThread();
        poseRenderer = std::make_unique<op::PoseCpuRenderer>(poseModel, flag_render_thrash, !flag_disable_blending);
        poseRenderer->initializationOnThread();

        // Measure execution time
        const auto now = std::chrono::high_resolution_clock::now();
        const auto totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now-timerBegin).count() * 1e-9;
        const auto message = "Initialized. Total time: " + std::to_string(totalTimeSec) + " seconds.";
        ROS_INFO_STREAM(message);
        init_finished = true;

        return 0;
}