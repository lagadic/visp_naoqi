/****************************************************************************
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
 * See http://team.inria.fr/lagadic/visp for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://team.inria.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * This example demonstrates how to get images from the robot remotely and how
 * to display them on your screen using ViSP.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*! \example image_viewer_visp.cpp */
#include <iostream>
#include <string>

/*

  KNOWN ISSUE DEALING WITH X11 and NAOQI

  If you get a strange compiler error on the line with None, like the following:

  naoqi-sdk-2.4.3.28-linux64/include/qi/type/detail/object.hxx:214:10: error: expected identifier before numeric constant
    struct None {
          ^
  it's probably because you're including X11 headers before NAOQI headers
  which defines also the symbol None.

  The workarround consists in including X11 includes after NAOQI ones; that's why
  vpNaoqiGrabber.h is included before vpDisplayX.h.

 */

#include <visp_naoqi/vpNaoqiGrabber.h>

#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>


const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d_%H.%M.%S", &tstruct);

  return buf;
}



/*!

   Connect to Nao or Romeo robot, grab and display images using ViSP.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./image_viewer_visp --ip <robot ip address>

   Example:

   ./image_viewer_visp --ip 169.254.168.230
 */
int main(int argc, const char* argv[])
{
  try
  {
    std::string opt_ip;
    int opt_cam = 0;
    bool opt_record = false;
    bool opt_VGA = false;

    for (unsigned int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--ip")
        opt_ip = argv[2];
      if (std::string(argv[i]) == "--cam")
        opt_cam = atoi(argv[i+1]);
      if (std::string(argv[i]) == "--record")
        opt_record = true;
     if (std::string(argv[i]) == "--vga")
       opt_VGA = true;
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--record] [--cam num_camera] [--help]" << std::endl;
        return 0;
      }
    }

    vpNaoqiGrabber g;
    g.setCamera(opt_cam); // left camera
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      g.setRobotIp(opt_ip);
    }

    if (opt_VGA)
         g.setCameraResolution(AL::kVGA);
    else
       g.setCameraResolution(AL::kQVGA);



    g.open();
    std::cout << "Open camera parameters: " << g.getCameraParameters() << std::endl;
    std::cout << "Dimension image: " << g.getHeight() <<"x" << g.getWidth() << std::endl;

    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    std::cout << "Extrinsic Camera parameters: " << g.get_eMc()<< std::endl;


    //Create directory
    std::string str_currentDate = currentDateTime();
    std::string output_img_dir = "images/" + str_currentDate + "/";
    if (opt_record)
    {
      std::cout << "Creating directory to store images" << std::endl;
      vpIoTools::makeDirectory("images");
      vpIoTools::makeDirectory("images/" + str_currentDate);
      vpIoTools::makeDirectory(output_img_dir);

    }

    unsigned int num_frame = 0;
    while(1)
    {
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);

      if (opt_record)
      {
        //Save frame
        char buffer[50];
        std::string format_name = "/image_%04d.png";
        sprintf (buffer, format_name.c_str(), num_frame);
        std::string filename = buffer;
        filename = output_img_dir + filename;
        vpImageIo::write(I, filename);
        num_frame++;
      }

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

    }
  }
  catch (const vpException &e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }
  catch (const AL::ALError &e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }

  return 0;
}

