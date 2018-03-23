## surveyor - v0.1.0

Surveyor is an independent software project implementing state-of-the-art visual odometry and 3D reconstruction algorithms. This package is currently designed to work with prerecorded image data from various sources and output a Point Cloud Data (PCD) file. Operation requires the **Robot Operating System (ROS)**, several C++ libraries (listed below), and a CUDA-capable GPU. This package has been tested on **Ubuntu 16.04** with **ros-kinetic**. Future plans are to pair the software in this package with live capture and computing hardware to create an untethered 3D modeling system that works entirely from 2D images without the need for separate depth sensing.

This package leverages two key algorithms: [**Direct Sparse Odometry (DSO)**](https://github.com/JakobEngel/dso) and [**Regularized Monocular Depth Estimation (REMODE)**](https://github.com/uzh-rpg/rpg_open_remode). In addition, custom functionality is included that helps to manage camera calibration information, serve prerecorded image data from either still images or video files, control the use of the aforementioned algorithms, and provide additional features to the end user.

The use of `surveyor` requires calibration information to accompany the input frames: both algorithms require camera calibration information for the capturing device, and DSO can optionally use additional photometric information for the camera. More details on acquiring geometric and photometric calibration information are included below. 

The final output of this system is a dense point cloud usable in architectural, engineering, or robotics applications.

*Note that `surveyor` is a Work-In-Progress and functionality may change as part of future updates. Express fitness for any particular task is also disclaimed.*


### Contents
1. [Dependencies](#dependencies)
2. [Installation](#installation)
3. [Theory of Operation](#theory-of-operation)
4. [Interfaces and Commands](#interfaces-and-commands)
5. [Using Surveyor](#using-surveyor)
6. [References](#references)
7. [License](#license)


### Dependencies

#### Required

- **opencv**: By default, OpenCV3 is included in `ros-kinetic` distribution.
- [**dso**](https://github.com/JakobEngel/dso#2-installation): Follow the instructions here to install and build DSO.
- [**rpg_open_remode**](Follow the instructions here to install and build REMODE. <NOTE ABOUT SVO>):

#### Optional

- **image_pipeline**: (CALIBRATION) If you have a set of images or a video file of a calibration sequence, you can use the `camera_calibration` node from the *image_pipeline* metapackage together with the `camera_emulator` to extract and save the calibration data to disk.


### Installation

1. Install and build all the required dependencies. Make note of where DSO and REMODE are stored and set the following environmental variables, replacing all bracketed (`<...>`) text with the relevant information:

```
export DSO_PATH=<PATH_TO_DSO_DIRECTORY>
export REMODE_PATH=<PATH_TO_REMODE_DIRECTORY>
```

2. Clone `surveyor` into the source directory of your ROS catkin workspace with `git clone https://github.com/spieswl/surveyor/`

3. Execute `catkin_make` in the root directory of your ROS catkin workspace.


### Theory of Operation

<TODO>

<SYSTEM DESIGN GOES HERE>


### Interfaces and Commands

#### camera_emulator.py

The `camera_emulator` node reads data (images, calibration, frame-by-frame poses) from a dataset directory specified at runtime. Several parameters are exposed for the user's benefit, detailed here:

- **function**: This parameter only has two options at present, `calibration` or `odometry`, which control the folder that images will be published from. No default value is assigned. NOTE that this parameter's setting is superseded if the **video** switch is set to `true`.

- **sequence**: This parameter allows the user to pick which dataset is to be drawn from. This parameter should be set to the name of a folder in the `./data/` directory inside the `surveyor` package. This value will be reused in the `surveyor-dso` and `surveyor-remode` nodes. No default value is assigned. More information on the use of this parameter is in the [Using Surveyor](#using-surveyor) section.

- **rate**: This parameter is a numerical value tied to the publishing frequency of the node. The default value of this parameter is **10**, as in 10 published images per second.

- **video**: This is a binary switch that allows the user to designate whether or not to process a video file (in lieu of a sequential set of images). As a result of OpenCV's limited video support, only **.AVI** files are supported. NOTE that setting this parameter to `true` will override the sub-directory behavior from the **function** parameter. The default value of this parameter is `false`. More information on the use of this parameter is in the [Using Surveyor](#using-surveyor) section.

##### Subscribing and Publishing

- This node does not subscribe to any topics.
- This node publishes image data on the `/camera_emu/image` topic ([sensor_msgs/Image.msg](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)).

##### Services

- This node provides a single service option, `/camera_emu/set_camera_info` ([sensor_msgs/SetCameraInfo](http://docs.ros.org/api/sensor_msgs/html/srv/SetCameraInfo.html)). This service waits for camera info to be supplied to the node via the service call; the node will record the passed camera calibration information to a file in the directory designated by the **sequence** parameter.


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

The `surveyor-remode` node is a ROS wrapper, written in C++, to REgularized MOnocular Depth Estimation.

<TODO>


### Using Surveyor

Surveyor can use prerecorded 2D images to serve as the input to the odometry-modeling (OM) pipeline. Additional components are needed in order to pass the image data and supply the necessary calibration information to the pipeline

1. `camera_emulator`
2. `surveyor_dso`           : Custom ROS wrapper to interface with DSO
3. `surveyor_remode`        : Custom ROS wrapper to interface with REMODE

x. `camera_calibration`     : Part of the `image_pipeline` metapackage (see __1. Dependencies__)

In addition to the 2D images, files containing the camera geometric calibration, gamma calibration, and a vignette sample should accompany the image set. Note that calibration images are unnecessary if geometric calibration information is already available to Surveyor. If using a prerecorded 2D image set, take care to arrange the information in the following fashion:

```
.
└── data
    └── <SEQUENCE>              # The sequence name should be passed as a parameter to the node.
        ├── images              # Pre-converted images should be stored here in numerical order.
        ├── calib               # Calibration images should be stored here.
        ├── camera.txt          # Geometric calibration information
        ├── pcalib.txt          # Gamma calibration information
        └── vignette.png        # Vignette sample
```

In addition to the *images* folder, *camera.txt*, *pcalib.txt*, and *vignette.png* are **REQUIRED** for Surveyor to run. The OM pipeline is prevented from working unless those files are present. More information on the calibration procedure can be found below.

<TODO>

#### Calibration Procedure

*Camera.txt* can be calculated by running the `camera_emulator` and `camera_calibration` nodes together; when the user interacts with the calibration software, the emulator will detect that `camera_calibration` is writing new CameraInfo to the emulated camera node and write the parameters (in the proper format for handling by the rest of the package code) to disk.

*Pcalib.txt* <TODO>

*Vignette.png* <TODO>

#### Relevant Launch Commands (WIP)

1. `rosrun surveyor camera_emulator.py _function:=<XXXX> _sequence:=<YYYY> _video:=<TRUE,FALSE>`
    * `_function` : Replace <XXXX> with 'calibration' or 'odometry', noting that the subfolder selected to pull images from will be different.
    * `_sequence` : Replace <YYYY> with the folder name of the dataset you wish Surveyor to use.
    * `_video` : If you want surveyor to run on a video file in the data directory, set this to 'true'.

2. `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.254 image:=/camera_emu/image camera:=/camera_emu`

3. `rosrun surveyor surveyor_dso source:=/camera_emu/image`

4. `rosrun surveyor surveyor_remode source:=/camera_emu/image`

5. `rosrun rviz rviz -d ~/Repositories/ros/src/surveyor/rviz/visualizer.rviz`

<TODO>

### References

<TODO>


### License

<TODO>