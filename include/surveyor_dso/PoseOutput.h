#ifndef __POSEOUTPUT_H
#define __POSEOUTPUT_H

#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include "FullSystem/HessianBlocks.h"
#include "IOWrapper/Output3DWrapper.h"
#include "util/FrameShell.h"
#include "util/MinimalImage.h"


namespace dso
{
    class CalibHessian;
    class FrameShell;

namespace IOWrap
{
    class PoseOutputWrapper : public Output3DWrapper
    {
    public:
        inline PoseOutputWrapper(std::string data_dir)
        {
            this->sequence_path_ = data_dir;
            this->pose_file_ = this->sequence_path_ + "/pose.txt";

            ROS_INFO_STREAM("SURVEYOR-DSO : Created PoseOutput I/O wrapper.");
            ROS_INFO_STREAM("SURVEYOR-DSO : Data directory path referenced to " << this->sequence_path_);

            this->pose_recorder_.open(this->pose_file_, std::ios::out);
        }

        virtual ~PoseOutputWrapper()
        {
            this->pose_recorder_.close();

            printf("SURVEYOR-DSO : Terminated PoseOutput I/O wrapper.\n");
        }

        virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override
        {
            // DEBUG
            ROS_INFO_STREAM("SURVEYOR-DSO : Current Frame " << frame->incoming_id);
            ROS_INFO_STREAM("SURVEYOR-DSO : (Time " << frame->timestamp << ", ID " << frame->id <<").");
            ROS_INFO_STREAM(frame->camToWorld.matrix3x4());
            // END DEBUG

            this->pose_recorder_ << frame->id << "\t\t";
            for (int k = 0; k < 3; k++)
            {
                this->pose_recorder_ << frame->camToWorld.matrix3x4().col(0)[k] << "\t";
                this->pose_recorder_ << frame->camToWorld.matrix3x4().col(1)[k] << "\t";
                this->pose_recorder_ << frame->camToWorld.matrix3x4().col(2)[k] << "\t";
                this->pose_recorder_ << frame->camToWorld.matrix3x4().col(3)[k] << "\t";
            }
            this->pose_recorder_ << "0.0\t0.0\t0.0\t1.0" << "\n";
        }

    private:
        std::string sequence_path_;
        std::string pose_file_;

        std::ofstream pose_recorder_;
    };
}
}
#endif