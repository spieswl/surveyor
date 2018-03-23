## surveyor - v0.1.0

Surveyor is an independent software project implementing state-of-the-art visual odometry and 3D reconstruction algorithms. This package is currently designed to work with prerecorded image data from various sources and output a Point Cloud Data (PCD) file. Operation requires the **Robot Operating System (ROS)**, several C++ libraries (listed below), and a CUDA-capable GPU. This package has been tested on **Ubuntu 16.04** with **ros-kinetic**. Future plans are to pair the software in this package with live capture and computing hardware to create an untethered 3D modeling system that works entirely from 2D images without the need for separate depth sensing.

This package leverages two key algorithms: [**Direct Sparse Odometry (DSO)**](https://github.com/JakobEngel/dso) and [**Regularized Monocular Depth Estimation (REMODE)**](https://github.com/uzh-rpg/rpg_open_remode). In addition, custom functionality is included that helps to manage camera calibration information, serve prerecorded image data from either still images or video files, control the use of the aforementioned algorithms, and provide additional features to the end user.

The use of `surveyor` requires calibration information to accompany the input frames: both algorithms require camera calibration information for the capturing device, and DSO can optionally use additional photometric information for the camera. More details on acquiring geometric and photometric calibration information are included below. 

The final output of this system is a dense point cloud usable in architectural, engineering, or robotics applications.

*Note that `surveyor` is a Work-In-Progress and functionality may change as part of future updates. Express fitness for any particular task is also disclaimed.*


## <LINE>

### Contents
1. [Dependencies](#dependencies)
2. [Installation](#installation)
3. [Sequence of Operation](#sequence-of-operation)
4. [Interfaces and Commands](#interfaces-and-commands)
5. [Using Surveyor](#using-surveyor)
6. [Results](#results)
7. [References](#references)
8. [License](#license)


### 1. Dependencies

#### Required

- **opencv**: By default, OpenCV3 is included in `ros-kinetic` distribution.
- [**dso**](https://github.com/JakobEngel/dso#2-installation): Follow the instructions here to install and build DSO.
- [**rpg_open_remode**](https://github.com/uzh-rpg/rpg_open_remode/wiki/Build-REMODE): Follow the instructions here to install and build REMODE. The ROS component is **REQUIRED** at present. This is one of the first things I would like to remove, which would eliminate any SVO dependencies.

#### Optional

- **image_pipeline**: (CALIBRATION) If you have a set of images or a video file of a calibration sequence, you can use the `camera_calibration` node from the *image_pipeline* metapackage together with the `camera_emulator` to extract and save the calibration data to disk.


### 2. Installation

1. Install and build all the required dependencies. Make note of where DSO and REMODE are stored and set the following environmental variables, replacing all bracketed (`<...>`) text with the relevant information:

```
export DSO_PATH=<PATH_TO_DSO_DIRECTORY>
export REMODE_PATH=<PATH_TO_REMODE_DIRECTORY>
```

2. Clone `surveyor` into the source directory of your ROS catkin workspace with `git clone https://github.com/spieswl/surveyor/`

3. Execute `catkin_make` in the root directory of your ROS catkin workspace.


### 3. Sequence of Operation

<TODO>

<SYSTEM DESIGN GOES HERE>


### 4. Interfaces and Commands

#### camera_emulator.py

The `camera_emulator` node reads data (images, calibration, frame-by-frame poses) from a dataset directory specified at runtime. Several parameters are exposed for the user's benefit, detailed here:

- **function**: This parameter only has two options at present, `calibration` or `odometry`, which control the folder that images will be published from. No default value is assigned. NOTE that this parameter's setting is superseded if the **video** switch is set to `true`.

- **sequence**: This parameter allows the user to pick which dataset is to be drawn from. This parameter should be set to the name of a folder in the `./data/` directory inside the `surveyor` package. This value will be reused in the `surveyor-dso` and `surveyor-remode` nodes. No default value is assigned. More information on the use of this parameter is in the [Using Surveyor](#using-surveyor) section.

- **video**: This is a binary switch that allows the user to designate whether or not to process a video file (in lieu of a sequential set of images). As a result of OpenCV's limited video support, only **.AVI** files are supported. NOTE that setting this parameter to `true` will override the sub-directory behavior from the **function** parameter. The default value of this parameter is `false`. More information on the use of this parameter is in the [Using Surveyor](#using-surveyor) section.

- **rate**: This parameter is a numerical value tied to the publishing frequency of the node. The default value of this parameter is **10**, as in 10 published images per second.

##### Subscribing and Publishing

- This node does not subscribe to any topics.
- This node publishes image data on the `/camera_emu/image` topic ([sensor_msgs/Image.msg](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)).

##### Services

- This node provides a single service, `/camera_emu/set_camera_info` ([sensor_msgs/SetCameraInfo](http://docs.ros.org/api/sensor_msgs/html/srv/SetCameraInfo.html)). This service waits for camera info to be supplied to the node via the service call; the node will record the passed camera calibration information to a file in the directory designated by the **sequence** parameter.


#### surveyor_dso.cpp

The `surveyor-dso` node is a ROS wrapper, written in C++, to Direct Sparse Odometry. By default, DSO boasts no ROS functionality, thus this code implements a simple class with a callback function (and other configuration controls) that leverages exposed functions in DSO. This node is tasked with feeding images to DSO, reading camera characteristics from the source dataset, recovering the calculated pose sequence for the capture device, and writing the frame-specific pose information to disk. Several parameters are exposed for the user's benefit, detailed here:

- **source**: This parameter allows the user to change the topic that this node subscribes to. The default value is `/camera_emu/image`.

- **visualizer**: This is a binary switch that, if set to `true`, enables the Pangolin visualization tool. The default value of this parameter is `false`.

##### Subscribing and Publishing

- This node subscribes to, by default, `/camera_emu/image` ([sensor_msgs/Image.msg](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)).
- This node does not publish on any topics.

##### Services

- This node maintains no services.


#### surveyor_remode.cpp

The `surveyor-remode` node is a ROS wrapper, written in C++, to Regularized Monocular Depth Estimation. This code is very similar in function to the DSO wrapper, but pulls from even more files housed in the target dataset. This node also relies on REMODE to create a topic for publishing the denoised and converged point cloud when the convergence threshold has been reached. As of the latest release, this code does not have any capability of its own to extract point clouds. Several parameters are exposed for the user's benefit, detailed here:

- **source**: This parameter allows the user to change the topic that this node subscribes to. The default value is `/camera_emu/image`.


##### Subscribing and Publishing

- This node subscribes to, by default, `/camera_emu/image` ([sensor_msgs/Image.msg](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)).
- This node does not publish on any topics.

- **NOTE**: Indirectly, `rpg_open_remode` will publish point clouds to a topic named `/remode/pointcloud` ([sensor_msgs/PointCloud](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html)).

##### Services

- This node maintains no services.


### 5. Using Surveyor

This package is designed to be an end-to-end model generation tool. Prerecorded 2D images serve as the main input to the odometry-modeling (OM) pipeline which, along with calibration information, allows the two algorithms to construct camera poses and dense point clouds, respectively. Subscribing to the `/remode/pointcloud` topic in RVIZ, or with the `pointcloud_to_pcd` node from the [***pcl_ros***](http://wiki.ros.org/pcl_ros) package, grants access to the point clouds generated by this system.

In addition to the 2D images, files containing the camera's geometric calibration, gamma or photometric calibration (if desired), and a vignette sample (if desired) should accompany the image set. If using a prerecorded 2D image set, take care to arrange the information in the following fashion:

```
.
└── data
    └── <SEQUENCE>              # The sequence name should be passed as a parameter to the 'camera_emulator' node.
        ├── images              # Compatible image files should be stored here in numerical order.
        ├── calib               # Calibration images should be stored here in numerical order.
        ├── camera.txt          # Geometric calibration information
        ├── distort.txt         # Camera distortion information
        ├── pcalib.txt          # Photometric calibration information
        ├── video.avi           # If using a video file, instead of 2D images, this file must be present.
        └── vignette.png        # Vignette sample
```

In addition to the *images* folder, *camera.txt* and *distort.txt* are **REQUIRED** for Surveyor to run. The OM pipeline is unable to develop point clouds unless those files are present. More information on the tasks necessary to generate those files can be found in [Calibration Procedures](#calibration-procedure).

*pcalib.txt* and *vignette.png* are used, if available, by Direct Sparse Odometry. While not required, they improve the accuracy of calculations performed by DSO to recover the camera pose for an arbitrary frame in the relayed image sequence. Other code, made available [here](https://github.com/tum-vision/mono_dataset_code) thanks to the Technical University of Munich, can help you determine those parameters.

The following commands are all that is necessary to operate `surveyor` from end-to-end.

1. `rosrun surveyor camera_emulator.py _function:=<XXXX> _sequence:=<YYYY> _video:=<TRUE,FALSE> _rate:=10`
    * `_function` : Replace <XXXX> with 'calibration' or 'odometry'.
    * `_sequence` : Replace <YYYY> with the folder name of the dataset you wish `surveyor` to use.
    * `_video` : If you want `surveyor` to run on the *video.avi* file in the designated data directory, set this to 'true'.
    * `_rate` : Numerical value of the publishing rate of the node.

2. `rosrun surveyor surveyor_dso _source:=/camera_emu/image _visualizer:=<TRUE,FALSE>`
    * `_source` : If you have a different topic as the image source, change this value.
    * `_visualizer` : If you wish to make use of the Pangolin visualizer to see the track of the camera, set this to 'true'.

3. `rosrun surveyor surveyor_remode _source:=/camera_emu/image`
    * `_source` : If you have a different topic as the image source, change this value.

4. `rosrun rviz rviz -d ./src/surveyor/rviz/visualizer.rviz`

An RVIZ configuration file is included for convenience in inspecting the output results. Point clouds will be generated in the default viewpoint of the camera.

Functionality to trigger graceful shutdowns on the part of `surveyor-dso` and `surveyor-remode` is not yet implemented. The user needs to send a `SIGINT` (**Ctrl-C**) to those nodes when the image sequence has completed. The camera emulator node will indicate in ROS Info logs when the sequence has finished.


#### 6. Calibration Procedures

Take special care to keep camera settings consistent across all calibration sequences. For reference, I used a Fujifilm X-T2 mirrorless digital camera for all calibration and image captures, which involved a lot of settings housekeeping. Industrial vision "ice cube" cameras are almost certainly easier to handle in this regard.

- *Camera.txt* and *distort.txt* can be calculated by running the `camera_emulator` and `camera_calibration` nodes together; when the user interacts with the calibration software, the emulator will detect that `camera_calibration` is writing new CameraInfo to the emulated camera node and write the parameters (in the proper format for handling by the rest of the package code) to disk.

For example, running `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.254 image:=/camera_emu/image camera:=/camera_emu` after using the default topic values, the `camera_emulator` node will take the calibration information passed in the SetCameraInfo service call and save the respective values to *camera.txt* and *distort.txt* in the sequence directory.

- *Pcalib.txt* is derived from running the TUM Mono Dataset code on a sequence of particular images. More information can be found on their [GitHub page](https://github.com/tum-vision/mono_dataset_code).

- *Vignette.png* is derived from running the TUM Mono Dataset code on a sequence of particular images. More information can be found on their [GitHub page](https://github.com/tum-vision/mono_dataset_code).


### 7. References

\[1\] **Direct Sparse Odometry**, *J. Engel, V. Koltun, D. Cremers*, in IEEE Transactions on Pattern Analysis and Machine Intelligence, March 2018.

\[2\] **REMODE: Probabilistic, Monocular Dense Reconstruction in Real Time**, *M. Pizzoli, C. Forster, and D. Scaramuzza,* in IEEE International Conference on Robotics and Automation (ICRA), 2014.


### 8. License

This package is released as open-source, and is licensed under the GNU General Public License Version 3 (GPLv3). 