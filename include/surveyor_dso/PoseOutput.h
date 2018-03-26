#ifndef __POSEOUTPUT_H
#define __POSEOUTPUT_H

#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include "FullSystem/HessianBlocks.h"
#include "IOWrapper/Output3DWrapper.h"
#include "util/FrameShell.h"
#include "util/MinimalImage.h"


class PoseOutputWrapper : public dso::IOWrap::Output3DWrapper
{
public:
    inline PoseOutputWrapper(std::string data_dir)
    {
        // File path variables
        this->sequence_path_ = data_dir;
        this->pose_file_ = this->sequence_path_ + "/pose.txt";

        ROS_INFO_STREAM("SURVEYOR-DSO : Created ROS interface node to DSO.");
        ROS_INFO_STREAM("SURVEYOR-DSO : Data directory path referenced to " << this->sequence_path_);

        this->pose_recorder_.open(this->pose_file_, std::ios::out);
    }

    virtual ~PoseOutputWrapper()
    {
        this->pose_recorder_.close();

        printf("SURVEYOR-DSO : Terminated ROS/DSO interface node.\n");
    }

    virtual void publishCamPose(dso::FrameShell* frame, dso::CalibHessian* HCalib) override
    {
        ROS_INFO_STREAM("SURVEYOR-DSO : Current Frame " << frame->incoming_id);
        ROS_INFO_STREAM("SURVEYOR-DSO : (Time " << frame->timestamp << ", ID " << frame->id <<").");

        // Pose output to file
        this->pose_recorder_ << frame->id << "\t\t";
        this->pose_recorder_ << std::fixed << std::setprecision(7);
        for (int k = 0; k < 3; k++)
        {
            this->pose_recorder_ << frame->camToWorld.matrix3x4().col(0)[k] << "\t";
            this->pose_recorder_ << frame->camToWorld.matrix3x4().col(1)[k] << "\t";
            this->pose_recorder_ << frame->camToWorld.matrix3x4().col(2)[k] << "\t";
            this->pose_recorder_ << frame->camToWorld.matrix3x4().col(3)[k] << "\t";
        }
        this->pose_recorder_ << std::setprecision(2);
        this->pose_recorder_ << float(0) << "\t" << float(0) << "\t" << float(0) << "\t" << float(1) << "\n";
    }

private:
    std::string sequence_path_;
    std::string pose_file_;

    std::ofstream pose_recorder_;
};

#endif