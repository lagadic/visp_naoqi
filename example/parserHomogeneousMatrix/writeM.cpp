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

/*! \example writeM.cpp */
#include <iostream>
#include <string>

#include<visp/vpXmlParserHomogeneousMatrix.h>


int main(int argc, char* argv[])
{
  try
  {
    //Create Pose Vector and convert to Homogeneous Matrix
    vpPoseVector r(1.0,1.3,3.5,0.2,0.3,0.5);
    vpHomogeneousMatrix M(r);

#ifdef VISP_HAVE_XML2
    vpXmlParserHomogeneousMatrix p; // Create a XML parser
    std::string name_M =  "eMc_CameraRigth_with_distorsion";
    char filename[FILENAME_MAX];
    sprintf(filename, "%s", "eMc_test.xml");

    if (p.save(M, filename, name_M) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK)
    {
      std::cout << "Cannot save the Homogeneous matrix" << std::endl;
    }


  }
  catch (const vpException &e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }

  vpXmlParser::cleanup();

#endif
  return 0;
}





