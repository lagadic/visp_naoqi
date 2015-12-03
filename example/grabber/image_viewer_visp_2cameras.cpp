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

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>

/*!

   Connect to Nao or Romeo robot, grab and display images from 2 cameras (on the front-head or
   on the eyes) using ViSP.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./image_viewer_visp_2cameras -ip <robot ip address>

   Example:

   ./image_viewer_visp_2cameras -ip 169.254.168.230
 */
int main(int argc, const char* argv[])
{
  try
  {
    std::string opt_ip;

    if (argc == 3) {
      if (std::string(argv[1]) == "-ip")
        opt_ip = argv[2];
    }

    vpNaoqiGrabber g;
    g.setCamerasMulti(0); // eyes cameras
    g.setCameraResolution(AL::kVGA);
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      g.setRobotIp(opt_ip);
    }

    g.openMulti();

    //Get cameras instrinsic parameters:
    vpCameraParameters caml = g.getCameraParameters(AL::kQVGA,"CameraLeftEye");
    vpCameraParameters camr = g.getCameraParameters(AL::kQVGA,"CameraRightEye");
    std::cout << "CameraLeftEye instrinsic parameters: " << std::endl << caml << std::endl;
    std::cout << "CameraRightEye instrinsic parameters: " << std::endl << camr << std::endl;


    //Get cameras extrinsic parameters:
    vpHomogeneousMatrix eMcl = g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraLeftEye");
    vpHomogeneousMatrix eMcr = g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraRightEye");
    std::cout << "CameraLeftEye extrinsic parameters: "  << std::endl << eMcl << std::endl;
    std::cout << "CameraRightEye extrinsic parameters: " << std::endl  << eMcr << std::endl;

    vpImage<unsigned char> Ir(g.getHeight(), g.getWidth());
    vpImage<unsigned char> Il(g.getHeight(), g.getWidth());

    vpDisplayX dr(Ir);
    vpDisplay::setTitle(Ir, "Right image");

    vpDisplayX dl(Il, Ir.getWidth()+40, 0);
    vpDisplay::setTitle(Il, "Left image");

   // std::cout << "Extrinsic Camera parameters: " << g.get_eMc()<< std::endl;
    vpMouseButton::vpMouseButtonType button;
    int iter = 1;
    while(1)
    {
      double t = vpTime::measureTimeMs();
      g.acquireMulti(Il,Ir);
      vpDisplay::display(Ir);
      vpDisplay::display(Il);

      vpDisplay::displayText(Il, 10, 10, "Left click to acquire, right to quit", vpColor::red);
      vpDisplay::displayText(Ir, 10, 10, "Left click to acquire, right to quit", vpColor::red);


      vpDisplay::flush(Ir);
      vpDisplay::flush(Il);

      if (vpDisplay::getClick(Ir, button, false)) {
        if (button == vpMouseButton::button1) {
          char name[100];
          sprintf(name, "left%02d.pgm", iter);
          vpImageIo::write(Il, name);
          sprintf(name, "right%02d.pgm", iter);
          vpImageIo::write(Ir, name);
          iter ++;
        }
        else if (button == vpMouseButton::button3) {
          break;
        }

      }
      if (vpDisplay::getClick(Il, button, false) ) {
        if (button == vpMouseButton::button1) {
          char name[100];
          sprintf(name, "left%02d.pgm", iter);
          vpImageIo::write(Il, name);
          sprintf(name, "right%02d.pgm", iter);
          vpImageIo::write(Ir, name);
          iter ++;
        }
        else if (button == vpMouseButton::button3) {
          break;
        }
      }

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

