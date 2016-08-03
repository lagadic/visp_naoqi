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

/*! \example image_viewer_opencv.cpp */
#include <iostream>
#include <string>

#include <visp_naoqi/vpNaoqiGrabber.h>


/*!

   Connect to Nao or Romeo robot, grab and display images using OpenCV.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./image_viewer_opencv --ip <robot ip address>

   Example:

   ./image_viewer_opencv -ip 169.254.168.230
 */
int main(int argc, const char* argv[])
{
  try
  {
    std::string opt_ip;

    if (argc == 3) {
      if (std::string(argv[1]) == "--ip")
        opt_ip = argv[2];
    }

    vpNaoqiGrabber g;
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      g.setRobotIp(opt_ip);
    }

    g.open();
    g.setCamera(0);
    g.setFramerate(3);


    std::cout << "Image size: " << g.getWidth() << " " << g.getHeight() << std::endl;
    // Create an OpenCV image container
    cv::Mat I = cv::Mat(cv::Size(g.getWidth(), g.getHeight()), CV_8UC3);

    // Create an OpenCV window to display the images
    cv::namedWindow("images");



    // Main loop. Exit when pressing ESC
    while (1) {
      double t = vpTime::measureTimeMs();

      char key = (char) cv::waitKey(30);
      g.acquire(I);

      // Display the image on screen
      cv::imshow("images", I);

      if (key == 32)
      {
        std::vector<int> compression_params; //vector that stores the compression parameters of the image

        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique

        compression_params.push_back(100); //specify the compression quality



        bool bSuccess = cv::imwrite("./TestImage.jpg", I, compression_params); //write the image to file



        if ( !bSuccess )

        {

          std::cout << "ERROR : Failed to save the image" << std::endl;

          //system("pause"); //wait for a key press

        }

      }



      if (key == 27)
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

