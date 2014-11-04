/****************************************************************************
 *
 * $Id: camera_calibration.cpp 4663 2014-02-14 10:32:11Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Camera calibration with chessboard or circle calibration grid.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#include <iostream>

#include <visp/vpConfig.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020300

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <visp/vpCalibration.h>

#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDot2.h>
#include <visp/vpIoTools.h>
#include <visp/vpPoint.h>
#include <visp/vpVideoReader.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpPose.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpXmlParserCamera.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

enum {
  NEW_IMAGE,
  CALIBRATE
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS

class Settings
{
public:
  Settings() {
    boardSize = cv::Size(0, 0);
    calibrationPattern = UNDEFINED;
    squareSize = 0.025f;
    goodInput = false;
    tempo = 1.f;

  }
  enum Pattern { UNDEFINED, CHESSBOARD, CIRCLES_GRID};

  bool read(const std::string &filename)    // Read the parameters
  {
    // reading configuration file
    if (! vpIoTools::loadConfigFile(filename) )
      return false;
    vpIoTools::readConfigVar("BoardSize_Width:", boardSize.width);
    vpIoTools::readConfigVar("BoardSize_Height:", boardSize.height);
    vpIoTools::readConfigVar("Square_Size:", squareSize);
    vpIoTools::readConfigVar("Calibrate_Pattern:", patternToUse);
    vpIoTools::readConfigVar("Input:", input);
    vpIoTools::readConfigVar("Tempo:", tempo);

    std::cout << "grid width : " << boardSize.width << std::endl;
    std::cout << "grid height: " << boardSize.height << std::endl;
    std::cout << "square size: " << squareSize << std::endl;
    std::cout << "pattern    : " << patternToUse << std::endl;
    std::cout << "input seq  : " << input << std::endl;
    std::cout << "tempo      : " << tempo << std::endl;
    interprate();
    return true;
  }
  void interprate()
  {
    goodInput = true;
    if (boardSize.width <= 0 || boardSize.height <= 0) {
      std::cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << std::endl;
      goodInput = false;
    }
    if (squareSize <= 10e-6) {
      std::cerr << "Invalid square size " << squareSize << std::endl;
      goodInput = false;
    }

    if (input.empty())      // Check for valid input
      goodInput = false;

    calibrationPattern = UNDEFINED;
    if (patternToUse.compare("CHESSBOARD") == 0) calibrationPattern = CHESSBOARD;
    else if (patternToUse.compare("CIRCLES_GRID") == 0) calibrationPattern = CIRCLES_GRID;
    if (calibrationPattern == UNDEFINED) {
      std::cerr << " Inexistent camera calibration mode: " << patternToUse << std::endl;
      goodInput = false;
    }
  }

public:
  cv::Size boardSize;        // The size of the board -> Number of items by width and height
  Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
  float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
  std::string input;         // The input image sequence
  float tempo;               // Tempo in seconds between two images. If > 10 wait a click to continue
  bool goodInput;

private:
  std::string patternToUse;
};

#endif
int main(int argc, const char ** argv)
{
  try {
    Settings s;
    const std::string inputSettingsFile = argc > 1 ? argv[1] : "default.cfg";
    if (! s.read(inputSettingsFile) ) {
      std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << std::endl;
      std::cout << std::endl << "Usage: " << argv[0] << " <configuration file>.cfg" << std::endl;
      return -1;
    }

    if (! s.goodInput)
    {
      std::cout << "Invalid input detected. Application stopping. " << std::endl;
      return -1;
    }

    // Start the calibration code
    vpImage<unsigned char> I;

    vpNaoqiRobot robot;
    robot.open();

    vpNaoqiGrabber g;
    g.open();
    g.acquire(I);

    char filename[FILENAME_MAX];
    vpCameraParameters cam;
    vpXmlParserCamera p; // Create a XML parser
    vpCameraParameters::vpCameraParametersProjType projModel; // Projection model
    // Use a perspective projection model without distortion
    projModel = vpCameraParameters::perspectiveProjWithDistortion;
    // Parse the xml file "myXmlFile.xml" to find the intrinsic camera
    // parameters of the camera named "myCamera" for the image sizes 640x480,
    // for the projection model projModel. The size of the image is optional
    // if camera parameters are given only for one image size.
    sprintf(filename, "%s", "camera.xml");
    if (p.parse(cam, filename, "Camera", projModel, I.getWidth(), I.getHeight()) != vpXmlParserCamera::SEQUENCE_OK) {
      std::cout << "Cannot found camera parameters in file: " << filename << std::endl;
      std::cout << "Loading default camera parameters" << std::endl;
      cam.initPersProjWithoutDistortion(342.82, 342.60, 174.552518, 109.978367);
    }

    std::cout << "Camera parameters: " << cam << std::endl;


#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined VISP_HAVE_GDI
    vpDisplayGDI d(I);
#elif defined VISP_HAVE_GTK
    vpDisplayGTK d(I);
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV d(I);
#endif

    std::vector<vpPoint> model;

    for (int i=0; i< s.boardSize.height; i++) {
      for (int j=0; j< s.boardSize.width; j++) {
        vpPoint P;
        P.setWorldCoordinates(j*s.squareSize, i*s.squareSize, 0);
        model.push_back(P);
      }
    }

    int nb_data = 0;
    int status = NEW_IMAGE;
    std::vector<vpHomogeneousMatrix> t_cMo;
    std::vector<vpHomogeneousMatrix> t_tMh;
    vpHomogeneousMatrix hMc;

    // Data acquisition
    while(status == NEW_IMAGE) {
      g.acquire(I);
      vpDisplay::display(I);
      vpMouseButton::vpMouseButtonType button;
      bool ret = vpDisplay::getClick(I, button, false);
      if (ret && button == vpMouseButton::button1) {
        status = NEW_IMAGE;
        sprintf(filename, "/tmp/I%03d.png", nb_data);
        vpImageIo::write(I, filename);



        std::vector<float> torsoMHead_ = robot.getProxy()->getTransform("HeadRoll", 0, true); // get torsoMHead of Aldebaran
        vpHomogeneousMatrix torsoMHead;
        unsigned int k=0;
        for(unsigned int i=0; i< 4; i++)
          for(unsigned int j=0; j< 4; j++)
            torsoMHead[i][j] = torsoMHead_[k++];

        std::cout << "Torso M Head:\n" << torsoMHead << std::endl;


        sprintf(filename, "/tmp/tMh_%03d.txt", nb_data);
        std::cout << "Save tMh to " << filename << std::endl;
        std::ofstream f(filename);
        torsoMHead.save(f);
        t_tMh.push_back(torsoMHead);

        nb_data ++;

      }
      else if (ret && button ==vpMouseButton::button3) {
        status = CALIBRATE;

      }
      vpDisplay::flush(I);
    }

    if (nb_data < 4) {
      std::cout << "Cannot calibrate: There are only " << nb_data << " images. There should be at least 4 images." << std::endl;
      return 0;
    }

    // Calibration
    for(int data_index=0; data_index < nb_data; data_index++) {

      sprintf(filename, "/tmp/I%03d.png", data_index);
      vpImageIo::read(I, filename);
      vpDisplay::display(I);

      std::cout << "Start calibration..." << std::endl;

      cv::Mat cvI;
      std::vector<cv::Point2f> pointBuf;
      vpImageConvert::convert(I, cvI);

      bool found = false;
      switch( s.calibrationPattern ) // Find feature points on the input format
      {
      case Settings::CHESSBOARD:
        //std::cout << "Use chessboard " << std::endl;
        found = findChessboardCorners( cvI, s.boardSize, pointBuf,
                                       CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        break;
      case Settings::CIRCLES_GRID:
        //std::cout << "Use circle grid " << std::endl;
        found = findCirclesGrid( cvI, s.boardSize, pointBuf, cv::CALIB_CB_SYMMETRIC_GRID  );
        break;
      case Settings::UNDEFINED:
      default:
        std::cout << "Unkown calibration grid " << std::endl;
        break;
      }

      std::cout << "frame: " << data_index << ", status: " << found;
      if (!found)
        std::cout << ", image rejected" << std::endl;
      else
        std::cout << ", image used as input data" << std::endl;

      if ( found)                // If done with success,
      {
        std::vector<vpImagePoint> data;

        if (s.calibrationPattern == Settings::CHESSBOARD) {
          // improve the found corners' coordinate accuracy for chessboard
          cornerSubPix( cvI, pointBuf, cv::Size(11,11),
                        cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
        }
        char title[20]; sprintf(title, "image %d", data_index);
        vpDisplay::setTitle(I, title);
        for (unsigned int i=0; i < pointBuf.size(); i++) {
          vpImagePoint ip(pointBuf[i].y, pointBuf[i].x);
          data.push_back(ip);
          vpDisplay::displayCross(I, ip, 10, vpColor::red);
        }

        // Calibration on a single mono image
        vpPose pose;
        vpPoint P;
        vpHomogeneousMatrix cMo;
        double x=0., y=0.;

        for (unsigned int i=0; i<model.size(); i++) {
          P.setWorldCoordinates(model[i].get_oX(), model[i].get_oY(), model[i].get_oZ());
          vpPixelMeterConversion::convertPoint(cam, data[i], x, y);
          P.set_x(x);
          P.set_y(y);
          pose.addPoint(P);
        }

        // Set (u0,v0) in the middle of the image
        //        double px = cam.get_px();
        //        double py = cam.get_py();
        //        double u0 = I.getWidth()/2;
        //        double v0 = I.getHeight()/2;
        //        cam.initPersProjWithoutDistortion(px, py, u0, v0);

        pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);
        pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
        t_cMo.push_back(cMo);

        std::cout << "Estimated pose on input data " << data_index << ": " << vpPoseVector(cMo).t() << std::endl;
        std::cout << "Estimated pose on input data:\n " << cMo << std::endl;
        sprintf(filename, "/tmp/cMo_%03d.txt", data_index);
        std::cout << "Save cMo to " << filename << std::endl;
        std::ofstream f(filename);
        cMo.save(f);
        vpDisplay::displayFrame(I, cMo, cam, 2*s.squareSize, vpColor::none, 3);
        vpDisplay::flush(I);
      }

      if (found)
        vpDisplay::displayCharString(I, 15, 15, "Image processing succeed", vpColor::green);
      else
        vpDisplay::displayCharString(I, 15, 15, "Image processing fails", vpColor::green);

      if (s.tempo > 10.f) {
        vpDisplay::displayCharString(I, 35, 15, "A click to process the next image", vpColor::green);
        vpDisplay::flush(I);
        vpDisplay::getClick(I);
      }
      else {
        vpDisplay::flush(I);
        vpTime::wait(s.tempo*1000);
      }
    }

    // Tsai calibration
    vpCalibration::calibrationTsai(t_cMo, t_tMh, hMc);
    std::cout << "Extrinsic camera parameters: " << hMc << std::endl;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
#else
int main()
{
  std::cout << "OpenCV 2.3.0 or higher is requested to run the calibration." << std::endl;
}
#endif
