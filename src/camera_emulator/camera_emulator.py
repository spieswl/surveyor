#!/usr/bin/env python

import rospy

import cv2
import cv_bridge
import glob
import rospkg
import sys

from sensor_msgs.msg import (Image, CameraInfo)

from sensor_msgs.srv import SetCameraInfo


class CameraEmulation():

    def __init__(self):

        # Publishers & Subscribers:
        self.emulated_feed = rospy.Publisher('camera_emu/image', Image, queue_size = 4)
        self.bridge = cv_bridge.CvBridge()

        # Service handling
        rospy.Service('camera_emu/set_camera_info', SetCameraInfo, self.svc_save_calib_info)

        # Functional parameters & variables
        if not rospy.has_param('~function'):
            rospy.loginfo("No 'function' parameter specified!")
            rospy.loginfo("USAGE: camera_emulator.py _function:=<OPTION> _sequence:=<FOLDER_NAME>")
            sys.exit(1)
        else:
            self.function = rospy.get_param('~function')

            if not self.function in ['calibration', 'odometry']:
                rospy.loginfo("Invalid 'function' parameter specified!")
                rospy.loginfo("FUNCTION: Please specify either \'calibration\' or \'odometry\'")
                sys.exit(1)

        if not rospy.has_param('~sequence'):
            rospy.loginfo("No 'sequence' parameter specified!")
            rospy.loginfo("USAGE: camera_emulator.py _function:=<OPTION> _sequence:=<FOLDER_NAME>")
            sys.exit(1)
        else:
            self.sequence_name = rospy.get_param('~sequence')

        self.package_path = rospkg.RosPack().get_path('surveyor')
        self.sequence_path = self.package_path + "/data/" + self.sequence_name

        self.out_framerate = rospy.Rate(5)                                         # Sticking with 5 fps for the moment
        self.out_width = 640
        self.out_height = 480

        self.calib_data = CameraInfo()
        self.calib_set = False


    def svc_save_calib_info(self, svc_request):

        self.calib_data = svc_request.camera_info

        # Completeness check -- If self.calib_data.K[0] == 0, assuming failure in calibration (or no calibration occurred)
        if self.calib_data.K[0] == 0:
            self.calib_set = False
            return (self.calib_set, "Camera calibration data INVALID.")

        else:
            # Use calibration data and save it in a format for use with DSO.
            # Camera intrinsic parameters
            calib_file = open(self.sequence_path + "/camera.txt","w+")

            calib_file.write(str( self.calib_data.K[0]/float(self.calib_data.width) ) + "\t" +
                             str( self.calib_data.K[4]/float(self.calib_data.height) ) + "\t" +
                             str( (self.calib_data.K[2] + 0.5)/float(self.calib_data.width) ) + "\t" +
                             str( (self.calib_data.K[5] + 0.5)/float(self.calib_data.height) ) + "\t" +
                             "0" + "\n")
            calib_file.write(str( self.calib_data.width ) + " " +
                             str( self.calib_data.height ) + "\n")
            calib_file.write("crop" + "\n")
            calib_file.write(str( self.out_width ) + " " +
                             str( self.out_height ) + "\n")

            calib_file.close()

            # Camera distortion parameters
            distort_file = open(self.sequence_path + "/distort.txt","w+")

            distort_file.write(str( self.calib_data.D[0] ) + "\t" +
                               str( self.calib_data.D[1] ) + "\t" +
                               str( self.calib_data.D[2] ) + "\t" +
                               str( self.calib_data.D[3] ) + "\n")

            distort_file.close()

            # Simple loop kickout for now, may have more sophisticated behavior based on this later
            self.calib_set = True

            return (self.calib_set, "Camera calibration data successfully applied.")


def main():

    # Initialization
    rospy.init_node('camera_emulator')
    emulator = CameraEmulation()

    main_iteration = 0;

    rospy.loginfo("Camera emulator initialized.")
    # DEBUG
    rospy.loginfo("Hit <ENTER> to continue...")
    raw_input()
    # END DEBUG

    # Command and control functionality
    if emulator.function == 'calibration':
        target = emulator.sequence_path + "/calib/*"
    elif emulator.function == 'odometry':
        target = emulator.sequence_path + "/images/*"

    # Main execution loop
    # //////////////////////////////////////////////////////////////////////
    while(True):
        if (main_iteration == 0):
            # DEBUG
            rospy.loginfo("DEBUG - Starting at head of image loop.")
            # END DEBUG

            # Grab all the images from the specified sequence directory in the ROS package
            for filename in sorted(glob.glob(target)):

                # DEBUG
                print filename
                # END DEBUG

                # Convert the OpenCV image to a ROS image via CV_Bridge
                src_image = cv2.imread(filename, 0)
                ros_image = emulator.bridge.cv2_to_imgmsg(src_image, encoding = "mono8")

                # Publish the converted image on the designated topic
                try:
                    emulator.emulated_feed.publish(ros_image)
                except emulator.bridge.CvBridgeError as err:
                    rospy.loginfo("ERROR - " + err)

                # Wait for the next frame to go out
                emulator.out_framerate.sleep()

            main_iteration += 1

        if (emulator.calib_set == True):
            rospy.loginfo("Camera calibration saved to disk - EXITING")
            break

    # DEBUG
    rospy.loginfo("DEBUG - Image sequence complete. Exiting...")
    # END DEBUG

    return
    # //////////////////////////////////////////////////////////////////////


if __name__ == '__main__':
    main()
