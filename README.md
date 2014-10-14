visp_naoqi
==========

visp_naoqi is a library that bridges ViSP and Naoqi.

# Prerequisities

- Install ViSP
- Install NaoQi

# How to build

	$ qibuild configure --release -c toolchain_romeo -DVISP_DIR=/local/soft/ViSP/ViSP-build-release
	$ qibuild make --release -c toolchain_romeo
