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


int main(int argc, char* argv[])
{
  try
  {
    vpNaoqiGrabber g;

    g.open();

    std::cout << "Image size: " << g.getWidth() << " " << g.getHeight() << std::endl;
    // Create an OpenCV image container
    cv::Mat I = cv::Mat(cv::Size(g.getWidth(), g.getHeight()), CV_8UC3);

    // Create an OpenCV window to display the images
    cv::namedWindow("images");

    // Main loop. Exit when pressing ESC
    while ((char) cv::waitKey(30) != 27) {

      double t = vpTime::measureTimeMs();
      g.acquire(I);

      // Display the image on screen
      cv::imshow("images", I);
      g.release();

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

