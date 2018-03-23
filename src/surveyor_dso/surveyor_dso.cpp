#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "util/settings.h"
#include "util/Undistort.h"
#include "FullSystem/FullSystem.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "surveyor_dso/PoseOutput.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "cv_bridge/cv_bridge.h"




using namespace dso;


// Global vars & constructs
FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 1;


void videoCallback(const sensor_msgs::ImageConstPtr inputImage)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(inputImage, sensor_msgs::image_encodings::MONO8);

    assert(cv_ptr->image.type() == CV_8U);
    assert(cv_ptr->image.channels() == 1);

    if(setting_fullResetRequested)
    {
        std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
        delete fullSystem;

        for(IOWrap::Output3DWrapper* ow : wraps)
        {
            ow->reset();
        }
        fullSystem = new FullSystem();
        fullSystem->linearizeOperation = false;
        fullSystem->outputWrapper = wraps;
        if(undistorter->photometricUndist != 0)
        {
            fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
        }
        setting_fullResetRequested = false;
    }

    MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows, (unsigned char*)cv_ptr->image.data);

    ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1, 0, 1.0f);

    fullSystem->addActiveFrame(undistImg, frameID);
    frameID++;

    delete undistImg;
}


int main(int argc, char** argv)
{
    // ROS node initialization
    ros::init(argc, argv, "surveyor_dso");
    ros::NodeHandle nh;
    ros::NodeHandle priv_param("~");

    // ROS Parameters
    // "Source"
    std::string source_param;
    priv_param.param<std::string>("source", source_param, "/camera_emu/image");

    // "Visualizer"
    bool visualizer_enabled;
    priv_param.param("visualizer", visualizer_enabled, false);

    // Check the ROS parameter server for "sequence", which will define the location of the data to check
    // for the calibration, gamma, and vignette files.
    std::string param_name, data_dir, sequence_name;
    std::string calibFile, gammaFile, vignetteFile;

    if(nh.searchParam("/camera_emulator/sequence", param_name))
    {
        nh.getParam(param_name, sequence_name);

        data_dir = ros::package::getPath("surveyor") + "/data/" + sequence_name;

        calibFile = data_dir + "/camera.txt";
        gammaFile = data_dir + "/pcalib.txt";
        vignetteFile = data_dir + "/vignette.png";
    }
    else
    {
        ROS_ERROR("SURVEYOR-DSO : No parameter named 'sequence'!");

        return 5;
    }

    // Hardcoded settings for "Surveyor" purposes, values are defaults from DSO's settings.cpp file unless otherwise noted.
    setting_debugout_runquiet = false;
    setting_desiredImmatureDensity = 1500;
    setting_desiredPointDensity = 2000;
    setting_kfGlobalWeight = 1.3;                   // Default in DSO is 1.0
    setting_logStuff = false;                       // Default in DSO is true
    setting_maxFrames = 7;
    setting_maxOptIterations = 6;
    setting_minFrames = 5;
    setting_minOptIterations = 1;

    setting_photometricCalibration = 0;             // Default in DSO is 2
    setting_useExposure = false;                    // Default in DSO is true
    setting_affineOptModeA = 0;                     // Default in DSO is 1e12
    setting_affineOptModeB = 0;                     // Default in DSO is 1e8

    // Undistort images based on established photometric calibration information
    undistorter = Undistort::getUndistorterForFile(calibFile, gammaFile, vignetteFile);
    setGlobalCalib((int)undistorter->getSize()[0], (int)undistorter->getSize()[1], undistorter->getK().cast<float>());

    fullSystem = new FullSystem();
    fullSystem->linearizeOperation = false;

    // Output components (hooked into DSO)
    // DEBUG - Enable visualization for now, eventually replaced with OutputWrapper functionality to store pose, etc.
    if (visualizer_enabled)
    {
        fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer((int)undistorter->getSize()[0], (int)undistorter->getSize()[1]));
    }
    // END DEBUG
    fullSystem->outputWrapper.push_back(new IOWrap::PoseOutputWrapper(data_dir));


    if(undistorter->photometricUndist != 0)
    {
        fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
    }

    // Core ROS infrastructure, connection to '/camera' or '/camera_emu' via the "source" parameter
    ros::Subscriber imgSub = nh.subscribe(source_param, 1, &videoCallback);

    ros::spin();

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
        delete ow;
    }

    delete undistorter;
    delete fullSystem;

    return 0;
}
