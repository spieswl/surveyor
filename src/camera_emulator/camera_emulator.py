#!/usr/bin/env python

import rospy
import rospkg

import cv2
import cv_bridge
import glob

from sensor_msgs.msg import (Image, CameraInfo)

from sensor_msgs.srv import SetCameraInfo


class CameraEmulation():

    def __init__(self):

        # Publishers and Subscribers:
        self.emulated_feed = rospy.Publisher('camera_emu/image', Image, queue_size = 4)
        self.bridge = cv_bridge.CvBridge()

        # Service Handling
        rospy.Service('camera_emu/set_camera_info', SetCameraInfo, self.svc_save_calib_info)

        # Class variables
        self.package_mgr = rospkg.RosPack()
        self.current_path = self.package_mgr.get_path('surveyor')
        self.img_calib_dir = self.current_path + "/data/sequence/calib/"
        self.img_seq_dir = self.current_path + "/data/sequence/images/"

        self.framerate = rospy.Rate(5)                                         # Stick with 5 fps for the moment

        self.output_width = 640
        self.output_height = 480

        self.calib_data = CameraInfo()
        self.calib_set = False


    def svc_save_calib_info(self, Request):

        # Assign service data to local variable
        self.calib_data = Request.camera_info

        # Completeness check -- If self.calib_data.K[0] == 0, assuming failure in calibration (or no calibration occurred)
        if self.calib_data.K[0] == 0:
            self.calib_set = False
            return (self.calib_set, "Camera calibration data INVALID.")
        else:
            # Use calibration data and save it in a format for use with DSO.
            calib_file = open(self.current_path + "/data/sequence/camera.txt","w+")

            calib_file.write("Pinhole" + "\t" +
                             str(self.calib_data.K[0]) + "\t" +
                             str(self.calib_data.K[4]) + "\t" +
                             str(self.calib_data.K[2]) + "\t" +
                             str(self.calib_data.K[5]) + "\t" +
                             "0" + "\n")
            calib_file.write(str(self.calib_data.width) + " " +
                             str(self.calib_data.height) + "\n")
            calib_file.write("crop" + "\n")
            calib_file.write(str(self.output_width) + " " +
                             str(self.output_height) + "\n")

            calib_file.close()

            self.calib_set = True

            return (self.calib_set, "Camera calibration data successfully applied.")


def main():

    # Node initialization
    rospy.init_node('camera_emulator')

    # Class initialization
    emulator = CameraEmulation()
    rospy.loginfo("Camera emulator initialized. Hit <ENTER> to continue...")
    raw_input()

    while(True):

        # DEBUG
        rospy.loginfo("DEBUG - Starting at head of image loop.")
        # END DEBUG

        # Grab all the images from the specified sequence directory in the ROS package
        for filename in sorted(glob.glob(emulator.img_calib_dir + "*")):

            # Access the images with OpenCV
            src_image = cv2.imread(filename, 0)

            # Convert the OpenCV image to a ROS image via CV_Bridge
            ros_image = emulator.bridge.cv2_to_imgmsg(src_image, encoding = "mono8")

            # Publish the converted image on the topic
            try:
                emulator.emulated_feed.publish(ros_image)
            except emulator.bridge.CvBridgeError as err:
                rospy.loginfo("ERROR - " + err)

            # Wait for the next frame to go out
            emulator.framerate.sleep()

        if (emulator.calib_set == True):
            rospy.loginfo("Camera calibration saved to disk - EXITING")
            break

    return


if __name__ == '__main__':
    main()
