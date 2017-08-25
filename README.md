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


### 1) System libraries conflict:
`$ qibuild configure -c toolchain_romeo -DVISP_DIR=/local/soft/ViSP/ViSP-build-release`

``` shell
CMake Warning at /udd/fspindle/.local/share/cmake/qibuild/target.cmake:85
(add_executable):
Cannot generate a safe runtime search path for target image_viewer_opencv
because files in some directories may conflict with libraries in implicit
directories:

runtime library [libz.so.1] in /usr/lib/x86_64-linux-gnu may be hidden by
files in:
      /local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib
```

In that case, backup `/local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib`
    and remove  the files`/naoqi-sdk-2.1.0.19-linux64/lib/libz.so.*`

```shell
  $ cd /local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib
  $ rm libz.so.*
```
    
With the `naoqi-sdk-2.3.0.14-linux64` we have another conflict with `[libusb-1.0.so.0]`. Remove  the files `libusb-1.*` in `/naoqi-sdk-2.3.0.14-linux64/lib`:

```shell
  $ cd /local/soft/romeo/devtools/naoqi-sdk-2.3.0.14-linux64/lib
  $ rm libusb-1.*
```
    
### 2) Macro names must be identifiers:

`$ qibuild make -c toolchain_romeo`

```
     ...
[ 20%] Building CXX object CMakeFiles/visp_naoqi.dir/src/grabber/vpNaoqiGrabber.cpp.o
<command-line>:0:1: error: macro names must be identifiers
<command-line>:0:1: error: macro names must be identifiers
<command-line>:0:1: error: macro names must be identifiers
```
 
Edit `/ViSP/ViSP-build-release/VISPConfig.cmake` to replace
    
`SET(VISP_DEFINITIONS "-DVP_TRACE;-DVP_DEBUG;-DUNIX")`

with:

`SET(VISP_DEFINITIONS "VP_TRACE;VP_DEBUG;UNIX")`
	
### 3) Conflicts with boost:

`$ qibuild make -c toolchain_romeo`

```
    Linking CXX executable sdk/bin/image_viewer_opencv
    /usr/bin/ld: warning: libboost_system.so.1.55.0, needed by   
    /local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib/libqitype.so, 
    may conflict with libboost_system.so.1.54.0
```

In that case, you have to build again ViSP turning Ogre support off:
`$ cd <ViSP build folder>`
`$ cmake -DUSE_OGRE=OFF <path to ViSP source code>`
`$ make -j8`

### 4)  metapod/algos/djac.hh: No such file or directory
* Open the file `CMakeLists.txt` of METAPOD and add the line:   
  `include/${PROJECT_NAME}/algos/djac.hh`   
in `SET(${PROJECT_NAME}_ALGOS_HEADERS` after `include/${PROJECT_NAME}/algos/jac_point_chain.hh`   
* Build and install Metapod again

### 5) ‘RotationMatrix’ is not a member of ‘metapod::Spatial’
If you have this error:

```shell
error: ‘RotationMatrix’ is not a member of ‘metapod::Spatial’
error: wrong number of template arguments (2, should be 3)
error: template argument 3 is invalid
error: ‘Joint’ does not name a type Joint joint;
```

* Rename
 `naoqi-sdk-2.3.0.14-linux64/bin/metapodfromurdf`
  to
 `/naoqi-sdk-2.3.0.14-linux64/bin/metapodfromurdf_aldebaran`
In this way CMake will find the right version of `metapodfromurdf` (in /usr/local/bin/metapodfromurdf).

### 6) CameraTop have already subscribed to the video device

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

