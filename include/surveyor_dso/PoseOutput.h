#ifndef __POSEOUTPUT_H
#define __POSEOUTPUT_H

#include <iostream>
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
            this->sequence_path = data_dir;
            this->pose_file = sequence_path + "/pose.txt";

            ROS_INFO_STREAM("SURVEYOR-DSO : Created PoseOutput I/O wrapper.");
            ROS_INFO_STREAM("SURVEYOR-DSO : Data directory path referenced to " << sequence_path);

            this->pose_recorder.open(this->pose_file, std::ios::out);
        }

        virtual ~PoseOutputWrapper()
        {
            this->pose_recorder.close();

            printf("SURVEYOR-DSO : Terminated PoseOutput I/O wrapper.\n");
        }

        virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override
        {
            // DEBUG
            ROS_INFO_STREAM("SURVEYOR-DSO : Current Frame " << frame->incoming_id);
            ROS_INFO_STREAM("SURVEYOR-DSO : (Time " << frame->timestamp << ", ID " << frame->id <<").");
            ROS_INFO_STREAM(frame->camToWorld.matrix3x4());
            // END DEBUG

            this->pose_recorder << frame->id << "\t\t";
            for (int k = 0; k < 3; k++)
            {
                this->pose_recorder << frame->camToWorld.matrix3x4().col(0)[k] << "\t";
                this->pose_recorder << frame->camToWorld.matrix3x4().col(1)[k] << "\t";
                this->pose_recorder << frame->camToWorld.matrix3x4().col(2)[k] << "\t";
                this->pose_recorder << frame->camToWorld.matrix3x4().col(3)[k] << "\t";
            }
            this->pose_recorder << "0.0\t0.0\t0.0\t1.0" << "\n";
        }

    private:
        // File manipulation variables for archiving of DSO pose results
        std::string sequence_path = "";
        std::string pose_file = "";
        std::ofstream pose_recorder;
    };
}
}
#endif