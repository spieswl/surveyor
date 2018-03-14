## surveyor - v0.1.0 (WORK IN PROGRESS)

Surveyor is an independent software project implementing state-of-the-art visual odometry and 3D reconstruction algorithms. This package can be used as an offline computation system, or it can be paired with specific hardware to create a live 3D modeling system that works entirely from 2D images. 

This package leverages two key algorithms: Direct Sparse Odometry (DSO) and Regularized Monocular Depth Estimation (REMODE). In addition, custom functionality is included that helps to manage camera calibration information, interface the two algorithms, and provide additional controls to the end user.

Surveyor can take sequences of prerecorded images as inputs. Each sequence requires some additional information to accompany the image set: photometric calibration data must be available for the leveraged algorithms to output accurate results, regardless of image quality.

The final output is a dense point cloud usable for architectural, engineering, or robotics applications.


### 1. Dependencies

<TODO>

1. opencv
2. [image_pipeline](https://github.com/ros-perception/image_pipeline)
3. [dso](https://github.com/JakobEngel/dso)
4. [rpg_open_remode](https://github.com/uzh-rpg/rpg_open_remode)


### 2. Installation

<TODO>


### 3. Theory of Operation & Conversion Pipeline 

<TODO>

<SYSTEM DESIGN GOES HERE>


### 4. Interface Description

#### camera_emulator.py

<TODO>


#### surveyor_dso.cpp

<TODO>


#### surveyor_remode.cpp

<TODO>


### 5. Using a Prerecorded 2D Image Dataset

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


#### Calibration Procedure

*Camera.txt* can be calculated by running the `camera_emulator` and `camera_calibration` nodes together; when the user interacts with the calibration software, the emulator will detect that `camera_calibration` is writing new CameraInfo to the emulated camera node and write the parameters (in the proper format for handling by the rest of the package code) to disk.

*Pcalib.txt* <TODO>

*Vignette.png* <TODO>


#### Relevant Launch Commands (WIP)

1. `rosrun surveyor camera_emulator.py _function:=<XXXX> _sequence:=<YYYY>`
    * `_function` : Replace <XXXX> with 'calibration' or 'odometry', noting that the subfolder selected to pull images from will be different.
    * `_sequence` : Replace <YYYY> with the folder name of the dataset you wish Surveyor to use.

2. `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.254 image:=/camera_emu/image camera:=/camera_emu`

3. `rosrun surveyor surveyor_dso source:=/camera_emu/image`

4. `rosrun surveyor surveyor_remode source:=/camera_emu/image`

5. `rosrun rviz rviz -d ~/Repositories/ros/src/surveyor/rviz/visualizer.rviz`


### 6. References

<TODO>