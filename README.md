visp_naoqi
==========

visp_naoqi is a library that bridges ViSP and Naoqi.

# Prerequisities

- Install ViSP: see <https://visp.inria.fr> and follow installation tutorials
- Install [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Install libqi:

        $ sudo apt-get install ros-kinetic-naoqi-libqi ros-kinetic-naoqi-libqicore

- Note also that in order to control Pepper in velocity with `visp_naoqi` you should install [pepper_control](https://github.com/lagadic/pepper_control) as a proxy in the robot

# How to build visp_naoqi

	$ git clone https://github.com/lagadic/visp_naoqi.git
	$ git checkout libqi
	$ mkdir build; cd build
	$ ccmake ../ -DVISP_DIR=/home/fspindle/soft/visp/visp-build -Dnaoqi_libqi_DIR=/opt/ros/kinetic/share/naoqi_libqi/cmake -Dnaoqi_libqicore_DIR=/opt/ros/kinetic/share/naoqi_libqicore/cmake/ -DCMAKE_INSTALL_PREFIX=/tmp/usr

# Known issues

## CameraTop have already subscribed to the video device

When in `Chroregraphe` you get the log :

    ERROR] vision.videodevice :_checkIsSubscriberValid:0 Subscriber "" not found.
    Please verify that you are subscribed to VideoDevice.
    [ERROR] vision.videodevice :_findAvailableSubscriberName:0 Could not subscribe module: CameraTop
    6 instances of: CameraTop have already subscribed to the video device.
    Please unsubscribe before subscribing again to this module.

Or when running `example/grabber/image_viewer_opencv` you get :

    $ ./image_viewer_opencv
    Connect to robot with ip address: 192.168.0.24
    [W] 1499867285.272633 13448 qi.path.sdklayout: No Application was created, trying to deduce paths
    Connected to a pepper robot.
    Cannot retrieve image
    Dimension image: 0x0
    Image size: 0 0
    Cannot retrieve image
    Cannot retrieve image
    OpenCV Error: Assertion failed (size.width>0 && size.height>0) in imshow, file /home/soft/opencv/   opencv-2.4.9-patched/modules/highgui/src/window.cpp, line 261
    terminate called after throwing an instance of 'cv::Exception'

It means that you have to unsuscribe ALVideoDevice

    $ ssh nao@192.168.0.24
    $ qicli info ALVideoDevice|grep subscribe
    $ qicli call ALVideoDevice.getSubscribers
    $ qicli call ALVideoDevice.unsubscribe CameraTop_0

## Build issue: macro names must be identifiers

When you get the following error:

	$ qibuild make -c toolchain_romeo
	...
	[ 20%] Building CXX object CMakeFiles/visp_naoqi.dir/src/grabber/vpNaoqiGrabber.cpp.o
	<command-line>:0:1: error: macro names must be identifiers
	<command-line>:0:1: error: macro names must be identifiers
	<command-line>:0:1: error: macro names must be identifiers

Modify /local/soft/ViSP/ViSP-build-release/VISPConfig.cmake the line:

	SET(VISP_DEFINITIONS "-DVP_TRACE;-DVP_DEBUG;-DUNIX")

with

	SET(VISP_DEFINITIONS "VP_TRACE;VP_DEBUG;UNIX")



