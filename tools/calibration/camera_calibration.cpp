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

#include <visp_naoqi/vpNaoqiGrabber.h>

#include <visp/vpCalibration.h>

#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpIoTools.h>
#include <visp/vpPoint.h>
#include <visp/vpVideoReader.h>
#include <visp/vpXmlParserCamera.h>



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
    std::string outputFileName = "camera.xml";

    //    std::cout << std::endl << "*********************************** " << std::endl;

    //    std::cout << std::endl << "Loading config file: " << std::endl;
    //    Settings s;
    //    const std::string inputSettingsFile = argc > 1 ? argv[1] : "default.cfg";
    //    if (! s.read(inputSettingsFile) ) {
    //      std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << std::endl;
    //      std::cout << std::endl << "Usage: " << argv[0] << " <configuration file>.cfg" << std::endl;
    //      return -1;
    //    }

    //    std::cout << std::endl << "*********************************** " << std::endl;

    //    if (! s.goodInput)
    //    {
    //      std::cout << "Invalid config file input detected. Application stopping. " << std::endl;
    //      return -1;
    //    }

    std::string opt_inputSettingsFile;
    std::string opt_ip;
    std::string camera_name = "camera";
    int opt_port = -1;
    int opt_cam = 0;
    bool opt_VGA = false;


    for (unsigned int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--config")
        opt_inputSettingsFile = argv[i+1];
      if (std::string(argv[i]) == "--ip")
        opt_ip = argv[i+1];
      if (std::string(argv[i]) == "--cam")
        opt_cam = atoi(argv[i+1]);
      if (std::string(argv[i]) == "--name")
        camera_name = argv[i+1];
      if (std::string(argv[i]) == "--vga")
        opt_VGA = true;
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << "  [ --config <configuration file>.cfg] [--ip <robot address>] [--cam camera_number] [--name camera_name] [--help] [--vga]" << std::endl;
        return 0;
      }
    }

    std::cout << std::endl << "*********************************** " << std::endl;

    std::cout << std::endl << "Loading config file: " << std::endl;
    Settings s;
    if (! s.read(opt_inputSettingsFile) ) {
      std::cout << "Could not open the configuration file: \"" << opt_inputSettingsFile << "\"" << std::endl;
      std::cout << std::endl << "Usage: " << argv[0] << " <configuration file>.cfg" << std::endl;
      return -1;
    }

    std::cout << std::endl << "*********************************** " << std::endl;

    if (! s.goodInput)
    {
      std::cout << "Invalid config file input detected. Application stopping. " << std::endl;
      return -1;
    }

    std::cout << "Camera name: " << camera_name << std::endl;
    std::cout << "Camera num: " << opt_cam << std::endl;

    // Start the calibration code
    vpImage<unsigned char> I;

    vpNaoqiGrabber g;
    g.setCamera(opt_cam);

    if (opt_VGA)
      g.setCameraResolution(AL::kVGA);
    else
      g.setCameraResolution(AL::kQVGA);

    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      g.setRobotIp(opt_ip);
    }
    if ( opt_port > 0) {
      std::cout << "Connect to robot with port: " << opt_port << std::endl;
      g.setRobotPort(opt_port);
    }

    g.open();
    g.acquire(I);
    std::cout << "Dimension image: " << g.getWidth() <<"x" << g.getHeight() << std::endl;

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
    std::vector<vpCalibration> calibrator;

    for (int i=0; i< s.boardSize.height; i++) {
      for (int j=0; j< s.boardSize.width; j++) {
        vpPoint P;
        P.setWorldCoordinates(j*s.squareSize, i*s.squareSize, 0);
        model.push_back(P);
      }
    }


    int nb_data = 0;
    int status = NEW_IMAGE;
    char filename[FILENAME_MAX];
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
        std::cout << "Image grabbed num: " << nb_data << std::endl;

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
        vpCalibration calib;
        calib.setLambda(0.5);
        calib.clearPoint();
        for (unsigned int i=0; i<model.size(); i++) {
          calib.addPoint(model[i].get_oX(), model[i].get_oY(), model[i].get_oZ(), data[i]);
        }
        vpHomogeneousMatrix cMo;
        vpCameraParameters cam;

        // Set (u0,v0) in the middle of the image
        double px = cam.get_px();
        double py = cam.get_py();
        double u0 = I.getWidth()/2;
        double v0 = I.getHeight()/2;
        cam.initPersProjWithoutDistortion(px, py, u0, v0);

        if (calib.computeCalibration(vpCalibration::CALIB_VIRTUAL_VS, cMo, cam, false) == 0) {
          //std::cout << "camera parameters: " << cam << std::endl;
          calibrator.push_back(calib);
        }
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

    // Now we consider the multi image calibration
    // Calibrate by a non linear method based on virtual visual servoing
    if (calibrator.empty()) {
      std::cerr << "Unable to calibrate. Image processing failed !" << std::endl;
      return 0;
    }

    std::cout << "\nCalibration without distorsion in progress on " << calibrator.size() << " images..." << std::endl;
    vpCameraParameters cam;
    double error;
    if (vpCalibration::computeCalibrationMulti(vpCalibration::CALIB_VIRTUAL_VS, calibrator, cam, error, false) == 0) {
      std::cout << cam << std::endl;
      std::cout << "Global reprojection error: " << error << std::endl;
#ifdef VISP_HAVE_XML2
      vpXmlParserCamera xml;

      if(xml.save(cam, outputFileName.c_str(), camera_name.c_str(), I.getWidth(), I.getHeight()) == vpXmlParserCamera::SEQUENCE_OK)
        std::cout << "Camera parameters without distortion successfully saved in \"" << outputFileName << "\"" << std::endl;
      else {
        std::cout << "Failed to save the camera parameters without distortion in \"" << outputFileName << "\"" << std::endl;
        std::cout << "A file with the same name exists. Remove it to be able to save the parameters..." << std::endl;
      }
#endif
    }
    else
      std::cout << "Calibration without distortion failed." << std::endl;

    std::cout << "\nCalibration with distorsion in progress on " << calibrator.size() << " images..." << std::endl;
    if (vpCalibration::computeCalibrationMulti(vpCalibration::CALIB_VIRTUAL_VS_DIST, calibrator, cam, error, false) == 0) {
      std::cout << cam << std::endl;
      std::cout << "Global reprojection error: " << error << std::endl;

#ifdef VISP_HAVE_XML2
      vpXmlParserCamera xml;

      if(xml.save(cam, outputFileName.c_str(), camera_name.c_str(), I.getWidth(), I.getHeight()) == vpXmlParserCamera::SEQUENCE_OK)
        std::cout << "Camera parameters without distortion successfully saved in \"" << outputFileName << "\"" << std::endl;
      else {
        std::cout << "Failed to save the camera parameters without distortion in \"" << outputFileName << "\"" << std::endl;
        std::cout << "A file with the same name exists. Remove it to be able to save the parameters..." << std::endl;
      }
#endif
      std::cout << std::endl;
    }
    else
      std::cout << "Calibration with distortion failed." << std::endl;

    return 0;
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
