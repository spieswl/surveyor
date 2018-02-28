## surveyor - v0.1.0 (WORK IN PROGRESS)

Surveyor is a combined hardware and software project implementing state-of-the-art visual odometry and 3D reconstruction algorithms. This package can be used as is, or can be paired with specific hardware to create a full 3D mapping system that works entirely with 2D images. 

This package leverages two key algorithms: Direct Sparse Odometry (DSO) and Regularized Monocular Depth Estimation (REMODE). In addition, custom functionality is included that helps to manage camera calibration information, interface the two algorithms, and provide additional controls to the end user.

Surveyor can take a sequence of prerecorded images, a prerecorded video file, or a live video feed as inputs. Each method requires some additional consideration: photometric calibration data must be available for the leveraged algorithms to output accurate results, regardless of input method.

The final output is a high-resolution model usable for architectural, engineering, or robotics applications.


### 1. Dependencies

1. opencv
2. [image_pipeline](https://github.com/ros-perception/image_pipeline)
3. [dso](https://github.com/JakobEngel/dso)
4. [rpg_open_remode](https://github.com/uzh-rpg/rpg_open_remode)


### 2. Theory of Operation & Conversion Pipeline 

PLACEHOLDER


### 3. Using a Prerecorded 2D Image Dataset

Surveyor can use prerecorded 2D images to serve as the input to the odometry-modelling (OM) pipeline. Additional components are needed in order to pass the image data and supply the necessary calibration information to the pipeline

1. `camera_emulator`
2. `camera_calibration`     : Part of the `image_pipeline` metapackage (see __1. Dependencies__)
3. `surveyor_dso`           : Custom ROS wrapper to interface with DSO
4. `TBD`

In addition to the 2D images, files containing the camera geometric calibration, gamma calibration, and a vignette sample should accompany the image set. If using a prerecorded 2D image set, take care to arrange the information in the following fashion:

```
.
└── data
    └── <SEQUENCE>              # The sequence name should be passed as a parameter to the node.
        ├── images              # Pre-converted images should be stored here in numerical order.
        ├── calib               # Calibration images should be stored here.
        ├── camera.txt          # Geometric calibration information
        ├── gamma.txt           # Gamma calibration information
        └── vignette.png        # Vignette sample
```

In addition to the *images* folder, *camera.txt*, *gamma.txt*, and *vignette.png* are **REQUIRED** for Surveyor to run. The OM pipeline is prevented from working unless those files are present. 

Calibration images are unnecessary if geometric calibration information is already available to Surveyor. *Camera.txt* can be calculated by running `camera_emulator` and `camera_calibration` together; when the user interacts with the calibration suite, the emulator will detect that `camera_calibration` is writing new CameraInfo to the emulated camera node and write the parameters (with proper formatting) to disk.


#### Relevant Launch Commands (WIP)

1. `rosrun surveyor camera_emulator.py _function:=<XXXX> _sequence:=<YYYY>`
    * `_function` : Replace <XXXX> with 'calibration' or 'odometry', noting that the dataset subfolder selected will be different.
    * `_sequence` : Replace <YYYY> with the folder name of the dataset you wish Surveyor to operate with.

2. `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.254 image:=/camera_emu/image camera:=/camera_emu`


### 4. References

PLACEHOLDER