#ifndef __DEPTHMAPOUTPUT_H
#define __DEPTHMAPOUTPUT_H

// Core C++ Includes
#include <future>
#include <fstream>
#include <iostream>
#include <sstream>

// REMODE Includes
#include <rmd/depthmap.h>
#include <rmd/publisher.h>

// ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


namespace surveyor
{
    namespace ProcessingStates
    {
        enum State
        {
            UPDATE,
            TAKE_REFERENCE_FRAME
        };
    }

    typedef ProcessingStates::State RemodeState;

    class RemodeNode
    {
    public:
        RemodeNode(ros::NodeHandle &nh, std::string data_dir);

        void initNode();
        void videoCallback(const sensor_msgs::ImageConstPtr &inputImage);

    private:
        ros::NodeHandle &nh_;

        std::string sequence_path_;
        std::string calib_file_;
        std::string distort_file_;
        std::string pose_file_;

        int frame_ID_;
        int num_msgs_;
        RemodeState state_;

        std::ifstream calib_reader_;
        std::ifstream distort_reader_;

        std::shared_ptr<rmd::Depthmap> depthmap_;
        std::unique_ptr<rmd::Publisher> publisher_;
        float ref_compl_perc_;
        float max_dist_from_ref_;
        int publish_conv_every_n_;

        float min_z_;
        float max_z_;

        void denoiseAndPublishResults();
        void publishConvergenceMap();
    };

}
#endif