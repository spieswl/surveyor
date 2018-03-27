#ifndef __POSEOUTPUT_H
#define __POSEOUTPUT_H

// Core C++ Includes
#include <fstream>
#include <iostream>

// DSO Includes
#include "FullSystem/HessianBlocks.h"
#include "IOWrapper/Output3DWrapper.h"
#include "util/FrameShell.h"
#include "util/MinimalImage.h"

// ROS Includes
#include <ros/ros.h>


namespace surveyor
{
    class PoseOutputWrapper : public dso::IOWrap::Output3DWrapper
    {
    public:
        PoseOutputWrapper(std::string data_dir);
        ~PoseOutputWrapper() override;

        void publishCamPose(dso::FrameShell* frame, dso::CalibHessian* HCalib) override;

    private:
        std::string sequence_path_;
        std::string pose_file_;

        std::ofstream pose_recorder_;
    };
}

#endif