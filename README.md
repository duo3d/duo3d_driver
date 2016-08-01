#DUO3D Driver for ROS 

The `duo3d_driver` ROS node interfaces with DUO SDK and publishes stereo images, disparity, point cloud, and IMU data from the DUO3D sensor. This package is build on Ubuntu 16.04.1 LTS x64 running ROS Kinetic.

##Introduction
The DUO MLX is an ultra-compact imaging sensor with global shutter and a standard USB interface for ease of use and connectivity. The DUO is intended for use in research, autonomous navigation, robotics and industrial areas. The camera's high speed and small size make it ideal for existing and new use cases for vision based applications.

![DUO MLX](https://duo3d.com/public/media/products/duo_m_mlx_feb10-136.png)

With a programmable illumination board and built-in IR filters and auto-exposure it allows for precise control of lighting environment, delivering configurable and precise stereo imaging for robotics, inspection, microscopy, visual odometry, human computer interaction and beyond.

The DUO MLX solution consists of:

 * Factory Calibrated Stereo Camera
 * Industrial Grade Monochrome/Global Shutter Sensors
 * Integrated Accelerometer/Gyroscope/Temperature (6 DoF IMU)
 * Fully Programmable Active LED Array (3xIR 850nm High Power LEDs)
 * DUO SDK License
 * DUO Dense3D License
 * USB Mini-B Cable

The DUO line of sensors offer a versatile board level solution for 3D sensing, a solution equipped with IMU and IR illumination and a board level sensor with an aluminum enclosure and a complete system for utilizing stereo vision that comes with a DUO MLX sensor and the DUO VPC processor. All DUO sensors are ready to work right out of the box and support a wide range of accessories and configurations.

For more information visit [DUO3D Home Page](https://duo3d.com)

##Installation

###DUO SDK
The DUO SDK package is required for DUO device support under your Linux operating system. It consists of kernel module and user mode libraries that expose all of the DUO functionalities with a simple and concise C/C++ API. Included in the package is the Dense3D&trade;, an SIMD/NEON optimized multi-threaded library that performs the disparity and 3D data extraction from DUO stereo image pair. 

 * To download the DUO SDK visit the [Download page](https://duo3d.com/docs/downloads)
 * To install the DUO SDK follow the [Install guide](https://duo3d.com/docs/articles/install-all)

###DUO ROS package
DUO ROS driver is developed and supported in ROS Kinetic. 

Get the package from github and put it in your catkin workspace ''src'' folder:

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/duo3d/duo3d_driver

Build the package:

    $ cd ~/catkin_ws
    $ catkin_make duo3d_driver
    $ source ./devel/setup.bash


###Published Topics
The `duo3d_driver` node interfaces with DUO SDK and publishes images, disparity, point cloud, and IMU data from the DUO3D sensor.

 * /duo3d_driver/left/image_rect (sensor_msgs/Image)
 Left camera rectified image
 * /duo3d_driver/left/camera_info (sensor_msgs/CameraInfo)
 Left camera info
 * /duo3d_driver/right/image_rect (sensor_msgs/Image)
 Right camera rectified image
 * /duo3d_driver/right/camera_info (sensor_msgs/CameraInfo)
 Right camera info
 * /duo3d_driver/rgb/image_rect (sensor_msgs/Image)
 Left camera RGB rectified image
 * /duo3d_driver/rgb/camera_info (sensor_msgs/CameraInfo)
 Left camera info
 * /duo3d_driver/depth/image_raw (sensor_msgs/Image)
 Colored disparity image
 * /duo3d_driver/depth/camera_info (sensor_msgs/CameraInfo)
 Disparity info
 * /duo3d_driver/point_cloud/image_raw (sensor_msgs/PointCloud2)
 DUO 3D point cloud data
 * /duo3d_driver/imu/data_raw (sensor_msgs/Imu)
 DUO IMU data

###Parameters
* `~frame_rate` (double, default: 30)
DUO image capture frame rate
* `~image_size` (vector<int>, default: {640,480})
DUO image frame size
* `~dense3d_license` (string)
Dense3D license string
* `~gain` (double, default: 0%)
Image gain value [0, 100]
* `~exposure` (double, default: 50%)
Image exposure value [0, 100]
* `~auto_exposure` (bool, default: False)
Image auto exposure value [False, True]
* `~camera_swap` (bool, default: False)
Image swap [False, True]
* `~horizontal_flip` (bool, default: False)
Horizontal image flip [False, True]
* `~vertical_flip` (bool, default: False)
Vertical image flip [False, True]
* `~led` (double, default: 10%)
LED brightness value [0, 100]
* `~accel_range` (int, default: +/-2g (0))
Accelerometer Range [+/-2g (0), +/-4g (1), +/-8g (2), +/-16g (3)]
* `~gyro_range` (int, default: 250deg/s)
Gyroscope Range [250deg/s (0), 500deg/s (1), 1000deg/s (2), 2000deg/s (3)]
* `~imu_rate` (double, default: 100Hz)
IMU Data Sampling Rate [25, 500]
* `~processing_mode` (int, default: BM (0))
Dense3D Processing Mode [BM (0), SGBM(1)]
* `~image_scale` (int, default: ScaleXY (3))
Dense3D Image Scaling Mode [ScaleNone (0), ScaleX (1), ScaleY (2), ScaleXY (3)]
* `~pre_filter_cap` (int, default: 28)
Dense3D Pre-filter cap [1, 63]
* `~num_disparities` (int, default: 4)
Dense3D Number of disparities [2, 16]
* `~sad_window_size` (int, default: 6)
Dense3D SAD Window Size [2, 10]
* `~uniqueness_ratio` (int, default: 27)
Dense3D Uniqueness Ratio [1, 100]
* `~speckle_window_size` (int, default: 52)
Dense3D Speckle Window Size [0, 256]
* `~speckle_range` (int, default: 14)
Dense3D Speckle Range [0, 32]

##Testing the DUO ROS package
Make sure that DUO device is plugged in the USB port and it is operating properly.

###DUO ROS Camera Example
This example demonstrates the acquisition of rectified stereo image pair from DUO. 
To launch DUO Camera ROS example, in a terminal run the following command:

    $ roslaunch duo3d_driver duo3d_camera.launch

You should see the following image:
![DUO ROS Camera Example](https://duo3d.com/public/media/products/ROS-DUO-Camera.jpg)

###DUO ROS IMU Example
This example demonstrates the DUO's on-board 6DoF sensor fusion using Madgwick algorithm running at 100Hz.
To launch DUO IMU ROS example, in a terminal run the following command:

    $ roslaunch duo3d_driver duo3d_imu.launch
 
You should see the following image:
![DUO ROS IMU Example](https://duo3d.com/public/media/products/ROS-DUO-IMU.jpg)

###DUO ROS Point Cloud Example
This example demonstrates disparity and point cloud generation using DUO. 
To launch DUO IMU Point Cloud example, in a terminal run the following command:

    $ roslaunch duo3d_driver duo3d_depth.launch
 
You should see the following image:
![DUO ROS Point Cloud Example](https://duo3d.com/public/media/products/ROS-DUO-PointCloud.jpg)

##Getting Help

 * For general help regarding DUO, you can visit the official [DUO forum](https://duo3d.com/forums)
 * For more information about DUO API please visit [DUO SDK](https://duo3d.com/docs/articles/sdk)
 * For ROS specific issues, please open a [ticket](https://github.com/duo3d/duo3d_driver/issues)