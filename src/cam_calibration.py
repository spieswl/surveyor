#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image, CameraInfo

from sensor_msgs.srv import SetCameraInfo


class CameraEmulator():

    def __init__(self):

        # Publishers and Subscribers:
        rospy.Publisher('camera_emu/image', Image, queue_size = 4)

        # Service Handling
        rospy.Service('camera_emu/set_camera_info', SetCameraInfo, self.svc_save_camera_info)





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass