#ifndef __POSEOUTPUT_H
#define __POSEOUTPUT_H

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
        inline PoseOutputWrapper()
        {
            printf("S-DSO : Created PoseOutput I/O wrapper.\n");
        }

        virtual ~PoseOutputWrapper()
        {
            printf("S-DSO : Terminated PoseOutput I/O wrapper.\n");
        }

        virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override
        {
            printf("S-DSO : Current Frame %d (Time %f, ID %d). Transform:\n", frame->incoming_id, frame->timestamp, frame->id);
            std::cout << frame->camToWorld.matrix3x4() << "\n\n";
        }

    };
}
}
#endif