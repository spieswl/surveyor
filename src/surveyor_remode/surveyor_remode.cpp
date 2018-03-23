#include "surveyor_remode/DepthmapOutput.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <rmd/depthmap.h>
#include <rmd/publisher.h>
#include <rmd/check_cuda_device.cuh>
#include <rmd/se3.cuh>


surveyor::RemodeNode::RemodeNode(ros::NodeHandle &nh, std::string data_dir) : nh_(nh)
{
    // File path variables
    this->sequence_path_ = data_dir;
    this->calib_file_ = sequence_path_ + "/camera.txt";
    this->distort_file_ = sequence_path_ + "/distort.txt";
    this->pose_file_ = sequence_path_ + "/pose.txt";

    this->frame_ID_ = 0;
    this->num_msgs_ = 0;
    this->state_ = surveyor::RemodeState::TAKE_REFERENCE_FRAME;

    // REMODE-specific settings
    // (Originals found in "rpg_open_remode/src/depthmap_node.cpp")
    this->ref_compl_perc_ = 15.0;                   // Higher is usually better, typ. 15.0
    this->max_dist_from_ref_ = 0.5;                 // Unknown
    this->publish_conv_every_n_ = 10;               // Unknown, doesn't seem to make difference, typ. 10

    // NOTE: "min_z" and "max_z" are based off of feature depths as determined from SVO.
    // This might be a problem without tailored numbers from DSO's pose calculation sequence.
    this->min_z_ = 0.0;                             // Typ. 0
    this->max_z_ = 100000.0;                        // Typ. 100,000

    ROS_INFO_STREAM("SURVEYOR-REMODE : Created ROS interface node to REMODE.");
    ROS_INFO_STREAM("SURVEYOR-REMODE : Data directory path referenced to " << this->sequence_path_);
}

void surveyor::RemodeNode::initNode()
{
    float cam_fx, cam_fy, cam_cx, cam_cy, omega;
    size_t cam_width, cam_height;
    float dist_k1, dist_k2, dist_p1, dist_p2;

    // ----------------------------------------
    // Extract camera calibration parameters from the supplied dataset
    this->calib_reader_.open(this->calib_file_, std::ios::in);

    this->calib_reader_ >> cam_fx;
    this->calib_reader_ >> cam_fy;
    this->calib_reader_ >> cam_cx;
    this->calib_reader_ >> cam_cy;
    this->calib_reader_ >> omega;             // Not currently used, retained for future use

    this->calib_reader_ >> cam_width;
    this->calib_reader_ >> cam_height;

    this->calib_reader_.close();
    // ----------------------------------------

    // Scaling to undo the normalization in the camera.txt file
    cam_fx = cam_fx * cam_width;
    cam_fy = cam_fy * cam_height;
    cam_cx = cam_cx * cam_width;
    cam_cy = cam_cy * cam_height;

    this->depthmap_ = std::make_shared<rmd::Depthmap>(cam_width, cam_height, cam_fx, cam_cx, cam_fy, cam_cy);

    // ----------------------------------------
    // Extract camera distortion parameters from the supplied dataset
    this->distort_reader_.open(this->distort_file_, std::ios::in);

    this->distort_reader_ >> dist_k1;
    this->distort_reader_ >> dist_k2;
    this->distort_reader_ >> dist_p1;
    this->distort_reader_ >> dist_p2;

    this->distort_reader_.close();
    // ----------------------------------------

    this->depthmap_->initUndistortionMap(dist_k1, dist_k2, dist_p1, dist_p2);

    // Initialize the built-in REMODE ROS publisher functionality
    this->publisher_.reset(new rmd::Publisher(this->nh_, this->depthmap_));
}

void surveyor::RemodeNode::videoCallback(const sensor_msgs::ImageConstPtr &inputImage)
{
    cv::Mat img_8UC1;
    float r[9] = { 0.0 };
    float t[3] = { 0.0 };

    std::ifstream pose_reader(this->pose_file_);
    std::string pose_line;
    int pose_ID = 0;
    bool valid_frame = false;

    this-> frame_ID_ += 1;
    this-> num_msgs_ += 1;

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(inputImage, sensor_msgs::image_encodings::MONO8);
    img_8UC1 = cv_ptr->image;

    ROS_INFO_STREAM("SURVEYOR-REMODE : Received " << img_8UC1.cols << "x" << img_8UC1.rows << " image.");

    // Extract pose information for the image frame of note by sweeping the pose file line-by-line
    // Package it up to send to the REMODE publisher
    while(std::getline(pose_reader, pose_line))
    {
        std::istringstream input_string(pose_line);

        input_string >> pose_ID;

        // Scan the pose file for the pose information corresponding to the currently accessed frame
        if(pose_ID != this->frame_ID_)
        {
            continue;
        }
        else
        {
            valid_frame = true;

            // Handles rows 1, 2, and 3 from pose file
            for(int k = 0; k < 3; k++)
            {
                input_string >> r[3*k];
                input_string >> r[3*k+1];
                input_string >> r[3*k+2];
                input_string >> t[k];
            }

            // DEBUG
            std::cout << "DEBUG - POSE ID : " << pose_ID << std::endl;
            // END DEBUG

            break;
        }
    }

    if(valid_frame)
    {
        rmd::SE3<float> T_world_curr(r, t);

        switch(this->state_)
        {
            case surveyor::RemodeState::TAKE_REFERENCE_FRAME:
            {
                if(this->depthmap_->setReferenceImage(img_8UC1, T_world_curr.inv(), this->min_z_, this->max_z_))
                {
                    this->state_ = RemodeState::UPDATE;
                }
                else
                {
                    ROS_ERROR("SURVEYOR-REMODE : Could not set reference image!");
                }
                break;
            }

            case surveyor::RemodeState::UPDATE:
            {
                this->depthmap_->update(img_8UC1, T_world_curr.inv());

                const float perc_conv = this->depthmap_->getConvergedPercentage();
                const float dist_from_ref = this->depthmap_->getDistFromRef();

                ROS_INFO_STREAM("SURVEYOR-REMODE : Percentage of converged measurements = " << perc_conv);

                if(perc_conv > this->ref_compl_perc_ || dist_from_ref > this->max_dist_from_ref_)
                {
                    this->state_ = RemodeState::TAKE_REFERENCE_FRAME;
                    denoiseAndPublishResults();
                }
                break;
            }

            default:
            {
                break;
            }
        }

        if(this->publish_conv_every_n_ < this->num_msgs_)
        {
            publishConvergenceMap();
            this->num_msgs_ = 0;
        }
    }
}

void surveyor::RemodeNode::denoiseAndPublishResults()
{
    this->depthmap_->downloadDenoisedDepthmap(0.5f, 500);               // Important values
    this->depthmap_->downloadConvergenceMap();

    std::async(std::launch::async, &rmd::Publisher::publishDepthmapAndPointCloud, *publisher_);
}

void surveyor::RemodeNode::publishConvergenceMap()
{
    this->depthmap_->downloadConvergenceMap();

    std::async(std::launch::async, &rmd::Publisher::publishConvergenceMap, *publisher_);
}


int main(int argc, char** argv)
{
    if(!rmd::checkCudaDevice(argc, argv))
    {
        ROS_ERROR("SURVEYOR-REMODE : Compatible CUDA device not found! Shutting down...");
        return 5;
    }

    std::string source_param;
    std::string param_name, data_dir, sequence_name;

    ros::init(argc, argv, "surveyor_remode");
    ros::NodeHandle nh;

    // ROS Parameters
    // "Source"
    nh.param<std::string>("source", source_param, "/camera_emu/image");

    // Check the ROS parameter server for "sequence", which will define the location of the data to check
    // for the calibration and pose files.
    if(nh.searchParam("/camera_emulator/sequence", param_name))
    {
        nh.getParam(param_name, sequence_name);

        data_dir = ros::package::getPath("surveyor") + "/data/" + sequence_name;
    }
    else
    {
        ROS_ERROR("SURVEYOR-REMODE : No parameter named 'sequence'!");
        return 5;
    }

    surveyor::RemodeNode srv_rmd_node(nh, data_dir);
    srv_rmd_node.initNode();

    ros::Subscriber imgSub = nh.subscribe(source_param, 1, &surveyor::RemodeNode::videoCallback, &srv_rmd_node);

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}