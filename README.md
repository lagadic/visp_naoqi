visp_naoqi
==========

visp_naoqi is a library that bridges ViSP and Naoqi.

# Prerequisities

- Install ViSP: see <http://team.inria.fr/lagadic/visp> and follow installation tutorials 
- [Install NaoQi](http://jokla.me/install-sdk-c-naoqi/)

# How to build in debug

	$ qibuild configure -c toolchain_romeo -DVISP_DIR=/local/soft/ViSP/ViSP-build-release
	$ qibuild make -c toolchain_romeo

# How to build in release

	$ qibuild configure --release -c toolchain_romeo -DVISP_DIR=/local/soft/ViSP/ViSP-build-release
	$ qibuild make --release -c toolchain_romeo


# Known issues

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



