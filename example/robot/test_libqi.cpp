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
 * Fabien Spindler
 *
 *****************************************************************************/

/*! \example motion_romeo.cpp */
#include <iostream>
#include <string>
#include <iterator>

#include <qi/session.hpp>
#include <qi/applicationsession.hpp>
#include <qi/anymodule.hpp>

#include "al/from_any_value.hpp"

#include "visp_naoqi/vpNaoqiRobot.h"
#include "vpFaceTrackerOkao.h"
/*!

   Connect to Pepper robot, and apply some motion.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./motion -ip <robot ip address>

   Example:

   ./motion -ip 131.254.10.126
 */
int main(int argc, char** argv)
{
  std::string opt_ip = "192.168.0.24";

    for (unsigned int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--ip")
        opt_ip = argv[i+1];
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << "[--ip <robot address>] " << std::endl;
        return 0;
      }
    }

  // Create a session to connect with the Robot
  qi::SessionPtr session = qi::makeSession();
  std::string ip_port = "tcp://" + opt_ip + ":9559";
  session->connect(ip_port);
  if (! opt_ip.empty()) {
    std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
  }

  vpNaoqiRobot robot(session);

  robot.open();

  std::vector<std::string> jointNames_head = robot.getBodyNames("Head");

  for (unsigned int i = 0; i<jointNames_head.size(); i++ )
    std::cout << jointNames_head[i] << std::endl;

  vpMatrix torso_eJe_head;
  robot.get_eJe("Head",torso_eJe_head);

  std::cout << "Jacobian Head: " << std::endl << torso_eJe_head << std::endl;

  vpFaceTrackerOkao face_tracker(session);

  std::vector <float> vel;
  vel.push_back(0.0);
  vel.push_back(0.1);

  while (1) {
    bool result = face_tracker.detect();
    if (result) {
      std::ostringstream text;
      text << "Found " << face_tracker.getNbObjects() << " face(s)";
    }

    robot.setVelocity(jointNames_head,vel);



  }


//  // Connect to module
//  qi::SessionPtr session = qi::makeSession();
//  session->connect("tcp://192.168.0.24:9559");
//  qi::AnyObject proxy = session->service("pepper_control");
//
//  // Start the controller
//  proxy.call<void >("start");
//
//  std::vector<std::string> jointNames_head(2);
//  jointNames_head[0] = "HeadYaw";
//  jointNames_head[1] = "HeadPitch";
//
//  std::vector<float> vel(2); // Fill with the joint velocities (rad/s)
//
//  vel[0] = 0.1;
//
//  while (1)
//  {
//    // Update velocities
//
//    proxy.async<void >("setDesJointVelocity", jointNames_head, vel );
//  }

// Stop the joint motion
  //proxy.call<void >("stopJoint");
//Stop the controller
 // proxy.call<void >("stop");


  return 0;



}
