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
 * This example demonstrates how to control the robot remotely in position and velocity.
 *
 * Authors:
 * Giovanni Claudio
 *
 *****************************************************************************/

/*! \example loadM.cpp */
#include <iostream>
#include <string>

#include<visp_naoqi/vpNaoqiConfig.h>
#include<visp/vpXmlParserHomogeneousMatrix.h>


int main(int argc, char* argv[])
{
  try
  {

    vpHomogeneousMatrix M;
#ifdef VISP_HAVE_XML2
    vpXmlParserHomogeneousMatrix p; // Create a XML parser

    std::string name =  "eMc_CameraLeft_with_distorsion";

    char filename[FILENAME_MAX];
    sprintf(filename, "%s", VISP_NAOQI_EXTRINSIC_CAMERA_FILE);

    if (p.parse(M,filename, name) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot found the Homogeneous matrix named " << name<< "." << std::endl;
      return 0;
    }
    else
     std::cout << "Homogeneous matrix " << name <<": " << std::endl << M << std::endl;

#endif

  }
  catch (const vpException &e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }


  return 0;
}

