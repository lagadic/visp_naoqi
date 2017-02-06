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

#include <visp_naoqi/vpNaoqiRobot.h>

#include <visp/vpMath.h>
#include <visp/vpTime.h>
#include <visp/vpColVector.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>
#include <visp/vpPlot.h>
#include <visp/vpDisplayX.h>

#include <qi/session.hpp>
#include <qi/applicationsession.hpp>
#include <qi/anymodule.hpp>

#include <iterator>

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

  //  {
  //    qi::SessionPtr session = qi::makeSession();
  //    session->connect("tcp://131.254.10.126:9559");
  //    qi::AnyObject proxy = session->service("ALMotion");

  //    proxy.async<void>("setAngles", "HeadYaw", -2.0 , 0.1);


  //    session->close();

  //    return 0;

  //  }


  {

    //AL::ALValue names       = AL::ALValue::array("HipRoll","HipPitch", "KneePitch","HeadYaw","HeadPitch" );
  //  AL::ALValue names1       = AL::ALValue::array("HipRoll","HipPitch", "KneePitch","HeadYaw","HeadPitch","RShoulderPitch","RShoulderRoll", "RElbowYaw", "RElbowRoll"); //,"RWristYaw","LShoulderPitch","LShoulderRoll", "LElbowYaw", "LElbowRoll","LWristYaw" );

//    AL::ALValue names_stiff       = AL::ALValue::array("Head","LArm","RArm" );




    //std::string opt_ip = "131.254.64.27";

    std::string opt_ip ="131.254.10.126";


    vpImage<unsigned char> I(320, 320);
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");


    vpNaoqiRobot robot;
    if (! opt_ip.empty())
      robot.setRobotIp(opt_ip);
    robot.open();

    if (robot.getRobotType() != vpNaoqiRobot::Pepper)
    {
      std::cout << "ERROR: You are not connected to Pepper, but to a different Robot. Check the IP. " << std::endl;
      return 0;
    }
    //robot.getProxy()->setStiffnesses(names_stiff, values_stiffnes);
    //robot.setPosition(names,values,1.0);
    // vpTime::sleepMs(1000);
    std::vector<std::string> names = robot.getBodyNames("Body");
    std::vector<float> values(names.size(),0.0);
    std::vector<float> values_stiffnes(3,1.0);

    robot.getProxy()->setMoveArmsEnabled(false,false);

    robot.getPosition(names,values,false);

    while(1)
    {

//      robot.setPosition(names,values,1.0);
//      //  robot.stop(names);

      robot.getProxy()->setAngles(names,values,1.0);
      robot.getProxy()->move(0.0, 0.0, -1.3);

      vpTime::sleepMs(20);


      if (vpDisplay::getClick(I, false))
        break;

    }

    robot.getProxy()->move(0.0, 0.0, 0.0);

    return 0;

  }






  {
    qi::SessionPtr session = qi::makeSession();
    session->connect("tcp://131.254.10.126:9559");
    qi::AnyObject proxy = session->service("pepper_control");

    proxy.call<void >("start");

    std::string name = "HeadYaw";

    vpImage<unsigned char> I(320, 320);
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    while(1)
    {
      proxy.async<void >("setOneDesJointVelocity", name, 0.05 );
      vpTime::sleepMs(50);

      if (vpDisplay::getClick(I, false))
        break;

    }
    proxy.call<void >("stopJoint");

    proxy.call<void >("stop");



  }


  return 0;




  {

    unsigned int joint_type = 0;


    /* 0 : HeadYaw, HeadPitch
   * 1: ShoulderPitch

  */

    vpNaoqiRobot robot;
    robot.setRobotIp("131.254.10.126");
    robot.open();

    vpImage<unsigned char> I(320, 320);
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");


    if (robot.getRobotType() != vpNaoqiRobot::Pepper)
    {
      std::cout << "ERROR: You are not connected to Pepper, but to a different Robot. Check the IP. " << std::endl;
      return 0;
    }

    std::vector<std::string> jointNames;


    switch (joint_type) {
    case 0:
      jointNames = robot.getBodyNames("Head");
      break;

    case 1:
      jointNames.push_back("LShoulderPitch");
      jointNames.push_back("RShoulderPitch");
      break;

    case 2:
      jointNames.push_back("LShoulderRoll");
      jointNames.push_back("RShoulderRoll");
      break;

    case 3:
      jointNames.push_back("LElbowRoll");
      jointNames.push_back("RElbowRoll");
      break;

    case 4:
      jointNames.push_back("LElbowYaw");
      jointNames.push_back("RElbowYaw");
      break;

    case 5:
      jointNames.push_back("LWristYaw");
      jointNames.push_back("RWristYaw");
      break;

    case 6:
      jointNames.push_back("HipRoll");
      jointNames.push_back("HeadYaw");
      break;

    case 7:
      jointNames.push_back("HipPitch");
      jointNames.push_back("HeadYaw");


      break;

    default:
      jointNames = robot.getBodyNames("Head");
      break;

    }



    // Plotting

    vpPlot plotter_diff_vel (2);
    plotter_diff_vel.initGraph(0, 2);
    plotter_diff_vel.initGraph(1, 2);
    plotter_diff_vel.setTitle(0,  jointNames[0].c_str());
    plotter_diff_vel.setTitle(1,  jointNames[1].c_str());

    vpPlot plotter_error (2);
    plotter_error.initGraph(0, 1);
    plotter_error.initGraph(1, 1);

    plotter_error.setTitle(0,  jointNames[0].c_str());
    plotter_error.setTitle(1,  jointNames[1].c_str());

    std::vector<float> jointVel( 2 );
    //    for (unsigned int i=0; i < jointVel.size(); i++)
    //      jointVel[i] = vpMath::rad(5);

    //    jointVel[0] = 20.25;// vpMath::rad(1000);

    unsigned long loop_iter = 0;
    vpColVector vel_head (jointNames.size());

    std::vector<float> q(jointNames.size());
    std::vector<float> q_new(jointNames.size());

    double t_prev = vpTime::measureTimeSecond();
    double start = t_prev;

    robot.setStiffness(jointNames, 1.0);

    double delta_t = 0.05;
    //while (vpTime::measureTimeSecond() < t_initial+10)

    qi::SessionPtr session = qi::makeSession();
    session->connect("tcp://131.254.10.126:9559");
    qi::AnyObject proxy = session->service("pepper_control");

    proxy.call<void >("start");


    bool first = true;
    bool second = true;
    double start_t = vpTime::measureTimeSecond();


    //    jointVel[0] = 0.1; //(sin(2*(now_t-start))); //vpMath::rad(10); // 20* (sin(2*(now_t-start)));
    //    jointVel[1] = 0.0;


    //    proxy.async<void >("setDesJointVelocity", jointNames, jointVel );

    while(1)
    {

      robot.getPosition(jointNames,q,true);

      // robot.setVelocity(jointNames, jointVel,true);
      double now_t = vpTime::measureTimeSecond();


      switch (joint_type) {
      case 0:
        jointVel[0] = 0.3* (sin(2*(now_t-start)));
        jointVel[1] = 0.3* (sin(2*(now_t-start)));
        break;

      case 1:
        jointVel[0] = 0.3* (sin(2*(now_t-start)));
        jointVel[1] = 0.3* (sin(2*(now_t-start)));
        break;
      case 2:
        jointVel[0] = 0.3* (sin(2*(now_t-start)));
        jointVel[1] = 0.3* (sin(2*(now_t-start)));
        break;
      case 3:
        jointVel[0] = 0.3* (sin(1.5*(now_t-start)));
        jointVel[1] = 0.3* (sin(1.5*(now_t-start)));
        break;
      case 4:
        jointVel[0] = 0.3* (sin(1.5*(now_t-start)));
        jointVel[1] = 0.3* (sin(1.5*(now_t-start)));
        break;
      case 5:
        jointVel[0] = 0.5* (sin(1.0*(now_t-start)));
        jointVel[1] = 0.5* (sin(1.0*(now_t-start)));
        break;
      case 6:
        jointVel[0] = 0.3* (sin(2.0*(now_t-start)));
        jointVel[1] = 0.0;
        break;
      case 7:
        jointVel[0] = 0.2* (sin(2.0*(now_t-start)));
        jointVel[1] = 0.0;
        break;
      default:
        break;

      }

      //      if (first) {
      proxy.async<void >("setDesJointVelocity", jointNames, jointVel );
      //        first = false;
      //      }

      //      if ((vpTime::measureTimeSecond() - start_t) > 1.) {
      //        start_t = vpTime::measureTimeSecond();
      //        jointVel[0] *= -1.;
      //        std::cout << " send new vel" << std::endl;
      //        proxy.async<void >("setDesJointVelocity", jointNames, jointVel );
      //      }

      vel_head = robot.getJointVelocity(jointNames);

      for (unsigned int i=0 ; i < jointNames.size() ; i++) {
        plotter_diff_vel.plot(i,1,loop_iter,jointVel[i] );//vpMath::rad(jointVel[i]));
        plotter_diff_vel.plot(i,0,loop_iter,vel_head[i]);

        plotter_error.plot(i,0,loop_iter,q[i]);
        loop_iter ++;
      }

      vpTime::sleepMs(50);


      if (vpDisplay::getClick(I, false))
        break;
    }

    proxy.call<void >("stop");


    robot.stop(jointNames);

    vpDisplay::getClick(I, true);

  }

  return 0;

  {


    vpNaoqiRobot robot;
    robot.setRobotIp("131.254.10.126");
    robot.open();

    vpImage<unsigned char> I(320, 320);
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");


    if (robot.getRobotType() != vpNaoqiRobot::Pepper)
    {
      std::cout << "ERROR: You are not connected to Pepper, but to a different Robot. Check the IP. " << std::endl;
      return 0;
    }

    std::vector<std::string> jointNames = robot.getBodyNames("Head");
    //    std::vector<std::string> jointNames;
    //    jointNames.push_back("LShoulderPitch");
    //    jointNames.push_back("RShoulderPitch");


    // Plotting

    vpPlot plotter_diff_vel (2);
    plotter_diff_vel.initGraph(0, 2);
    plotter_diff_vel.initGraph(1, 2);
    plotter_diff_vel.setTitle(0,  jointNames[0].c_str());
    plotter_diff_vel.setTitle(1,  jointNames[1].c_str());

    vpPlot plotter_error (2);
    plotter_error.initGraph(0, 1);
    plotter_error.initGraph(1, 1);

    plotter_error.setTitle(0,  jointNames[0].c_str());
    plotter_error.setTitle(1,  jointNames[1].c_str());

    std::vector<float> jointVel( 2 );
    //    for (unsigned int i=0; i < jointVel.size(); i++)
    //      jointVel[i] = vpMath::rad(5);

    //    jointVel[0] = 20.25;// vpMath::rad(1000);

    unsigned long loop_iter = 0;
    vpColVector vel_head (jointNames.size());

    std::vector<float> q(jointNames.size());
    std::vector<float> q_new(jointNames.size());

    double t_prev = vpTime::measureTimeSecond();
    double start = t_prev;

    robot.setStiffness(jointNames, 1.0);

    double delta_t = 0.05;
    //while (vpTime::measureTimeSecond() < t_initial+10)

    qi::SessionPtr session = qi::makeSession();
    session->connect("tcp://131.254.10.126:9559");
    qi::AnyObject proxy = session->service("pepper_control");

    proxy.call<void >("start");


    bool first = true;
    bool second = true;
    double start_t = vpTime::measureTimeSecond();


    //    jointVel[0] = 0.1; //(sin(2*(now_t-start))); //vpMath::rad(10); // 20* (sin(2*(now_t-start)));
    //    jointVel[1] = 0.0;


    //    proxy.async<void >("setDesJointVelocity", jointNames, jointVel );

    while(1)
    {

      robot.getPosition(jointNames,q,true);

      // robot.setVelocity(jointNames, jointVel,true);
      double now_t = vpTime::measureTimeSecond();

      jointVel[0] = 0.0;// (sin(2*(now_t-start))); //vpMath::rad(10); // 20* (sin(2*(now_t-start)));
      jointVel[1] = 0.8* (sin(2*(now_t-start)));

      //      if (first) {
      proxy.async<void >("setDesJointVelocity", jointNames, jointVel );
      //        first = false;
      //      }

      //      if ((vpTime::measureTimeSecond() - start_t) > 1.) {
      //        start_t = vpTime::measureTimeSecond();
      //        jointVel[0] *= -1.;
      //        std::cout << " send new vel" << std::endl;
      //        proxy.async<void >("setDesJointVelocity", jointNames, jointVel );
      //      }

      vel_head = robot.getJointVelocity(jointNames);

      for (unsigned int i=0 ; i < jointNames.size() ; i++) {
        plotter_diff_vel.plot(i,1,loop_iter,jointVel[i] );//vpMath::rad(jointVel[i]));
        plotter_diff_vel.plot(i,0,loop_iter,vel_head[i]);

        plotter_error.plot(i,0,loop_iter,q[i]);
        loop_iter ++;
      }

      vpTime::sleepMs(50);


      if (vpDisplay::getClick(I, false))
        break;
    }

    proxy.call<void >("stop");


    robot.stop(jointNames);

    vpDisplay::getClick(I, true);

  }

















  //  {
  //    // Plotting

  //    vpPlot plotter_diff_vel (2);
  //    plotter_diff_vel.initGraph(0, 2);
  //    plotter_diff_vel.initGraph(1, 2);
  //    plotter_diff_vel.setTitle(0,  "HeadYaw");
  //    plotter_diff_vel.setTitle(1,  "HeadPitch");

  //    vpPlot plotter_error (2);
  //    plotter_error.initGraph(0, 1);
  //    plotter_error.initGraph(1, 1);

  //    plotter_error.setTitle(0,  "HeadYaw");
  //    plotter_error.setTitle(1,  "HeadPitch");

  //    vpNaoqiRobot robot;
  //    robot.setRobotIp("131.254.10.126");
  //    robot.open();

  //    vpImage<unsigned char> I(320, 320);
  //    vpDisplayX d(I);
  //    vpDisplay::setTitle(I, "ViSP viewer");


  //    if (robot.getRobotType() != vpNaoqiRobot::Pepper)
  //    {
  //      std::cout << "ERROR: You are not connected to Pepper, but to a different Robot. Check the IP. " << std::endl;
  //      return 0;
  //    }

  //    std::vector<std::string> jointNames_head = robot.getBodyNames("Head");

  //    std::vector<float> jointVel( 2 );
  ////    for (unsigned int i=0; i < jointVel.size(); i++)
  ////      jointVel[i] = vpMath::rad(5);

  //    jointVel[0] = 20.25;// vpMath::rad(1000);

  //    unsigned long loop_iter = 0;
  //    vpColVector vel_head (jointNames_head.size());

  //    std::vector<float> q(jointNames_head.size());
  //    std::vector<float> q_new(jointNames_head.size());

  //    double t_prev = vpTime::measureTimeSecond();

  //    std::cout << "vpTime:" << t_prev << std::endl;

  //    robot.setStiffness(jointNames_head, 1.0);

  //    double delta_t = 0.0;
  //    //while (vpTime::measureTimeSecond() < t_initial+10)
  //    while(1)
  //    {

  //     robot.getPosition(jointNames_head,q,true);

  //      // robot.setVelocity(jointNames_head, jointVel,true);
  //     double now_t = vpTime::measureTimeSecond();
  //     delta_t = now_t - t_prev;

  //     jointVel[0] = 40.25 * (sin(0.001*loop_iter));

  //     for (unsigned int i=0 ; i < jointNames_head.size() ; i++) {
  //      q_new[i] = q[i] +  jointVel[i]*delta_t;

  //     }

  //     std::cout << "Pose:" << q_new << std::endl;
  //     std::cout << "Delta:" << delta_t << std::endl;

  //     robot.setPosition(jointNames_head,q_new,1.0);

  //     t_prev = now_t;

  //      vel_head = robot.getJointVelocity(jointNames_head);
  //      for (unsigned int i=0 ; i < jointNames_head.size() ; i++) {
  //        plotter_diff_vel.plot(i,1,loop_iter,jointVel[i]/100);
  //        plotter_diff_vel.plot(i,0,loop_iter,vel_head[i]);

  //        plotter_error.plot(i,0,loop_iter,vel_head[i]);
  //        loop_iter ++;
  //      }


  //      if (vpDisplay::getClick(I, false))
  //        break;
  //    }

  //    robot.stop(jointNames_head);

  //        vpDisplay::getClick(I, true);

  //  }






  //  try {


  //    qi::ApplicationSession app(argc, argv);
  //    app.start();
  //    qi::SessionPtr session = app.session();
  //    qi::AnyObject tts = session->service("ALTextToSpeech");
  //    tts.call<void>("say", "Hello world!");

  // motion.async<void>("setAngles", "HeadYaw", 0.0, 0.3);






  //{
  //  qi::ApplicationSession app(argc, argv);
  //     app.start(); // connect the session
  //     qi::SessionPtr session = app.session();
  //     qi::AnyObject motion = session->service("ALMotion");

  //     motion.async<void>("changeAngles", "HeadYaw", -0.7, 0.3);

  //     return 0;
  //}

  //  qi::ApplicationSession app(argc, argv);
  //  app.start(); // connect the session
  //  qi::SessionPtr session = app.session();
  //  qi::AnyObject proxy = session->service("ALMotion");

  /*  qi::SessionPtr session = qi::makeSession();
  session->connect("tcp://131.254.10.126:9559");
  qi::AnyObject proxy = session->service("ALMotion")*/;


  //  AL::ALValue names =  AL::ALValue::array("HeadYaw", "HeadPitch");
  //  AL::ALValue angles      = AL::ALValue::array(0.0f, -0.0f);
  ////  AL::ALValue vel      = AL::ALValue::array(0.3f, -0.3f);

  //  std::vector<float> vel(2);
  //  vel[0] = 0.2;
  //  vel[1] = 0.2;
  //  proxy.async<void>("setAngles", names, angles, vel);

  //qi::os::sleep(3.0f);

  //    app.session()->close();

  //  try
  //  {
  //    std::string opt_ip;

  //    if (argc == 3) {
  //      if (std::string(argv[1]) == "--ip")
  //        opt_ip = argv[2];
  //    }



  //    /** The name of the joint to be moved. */
  //    const AL::ALValue jointName = "HeadYaw";


  //    try {
  //      /** Create a ALMotionProxy to call the methods to move NAO's head.
  //      * Arguments for the constructor are:
  //      * - IP adress of the robot
  //      * - port on which NAOqi is listening, by default 9559
  //      */
  //      AL::ALMotionProxy motion("131.254.10.126", 9559);

  //      /** Make sure the head is stiff to be able to move it.
  //      * To do so, make the stiffness go to the maximum in one second.
  //      */
  //      /** Target stiffness. */
  //      AL::ALValue stiffness = 1.0f;
  //      /** Time (in seconds) to reach the target. */
  //      AL::ALValue time = 1.0f;
  //      /** Call the stiffness interpolation method. */
  //      motion.stiffnessInterpolation(jointName, stiffness, time);

  //      /** Set the target angle list, in radians. */
  //      AL::ALValue targetAngles = AL::ALValue::array(-1.5f, 1.5f, 0.0f);
  //      /** Set the corresponding time lists, in seconds. */
  //      AL::ALValue targetTimes = AL::ALValue::array(3.0f, 6.0f, 9.0f);
  //      /** Specify that the desired angles are absolute. */
  //      bool isAbsolute = true;

  //      /** Call the angle interpolation method. The joint will reach the
  //      * desired angles at the desired times.
  //      */
  //      //motion.angleInterpolation(jointName, targetAngles, targetTimes, isAbsolute);

  //      motion.setAngles(jointName, -1.5f, 0.5);


  //    }
  //    catch (const AL::ALError& e) {
  //      std::cerr << "Caught exception: " << e.what() << std::endl;
  //      exit(1);
  //    }

  //    return 0;

  //  {
  //      AL::ALMotionProxy motion("131.254.10.126");

  //      // Setting head stiffness on.
  //       motion.setStiffnesses("Head", 1.0f);

  //       // Example showing how to set angles, using a fraction of max speed
  //       AL::ALValue names       = AL::ALValue::array("HeadYaw", "HeadPitch");
  //       AL::ALValue angles      = AL::ALValue::array(0.3f, -0.3f);
  //       float fractionMaxSpeed  = 0.1f;
  //       motion.setStiffnesses(names, AL::ALValue::array(1.0f, 1.0f));
  //       qi::os::sleep(1.0f);
  //       motion.setAngles(names, angles, fractionMaxSpeed);
  //       qi::os::sleep(1.0f);

  //       std::cout << "test" << std::endl;

  //      return 0;

  //    }



  //  vpNaoqiRobot robot;
  //  if (! opt_ip.empty()) {
  //    std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
  //    robot.setRobotIp(opt_ip);
  //  }
  //  else
  //    robot.setRobotIp("131.254.10.126");

  //  robot.open();


  //  if (robot.getRobotType() != vpNaoqiRobot::Pepper)
  //  {
  //    std::cout << "ERROR: You are not connected to Pepper, but to a different Robot. Check the IP. " << std::endl;
  //    return 0;
  //  }


  //  //Get the actual Jacobian of the Head
  //  std::string names = "Head";



  //  std::cout << "Test " << names << " retrieving joint positions" << std::endl;

  //  vpColVector jointPositions = robot.getPosition(names);
  //  std::vector<std::string> jointNames = robot.getBodyNames(names);
  //  for (unsigned int i=0; i< jointPositions.size(); i++)
  //    std::cout << "Sensor " << jointNames[i] << " position: " << jointPositions[i] << std::endl;

  //  std::cout << "Test " << names << " Jacobian" << std::endl;
  //  vpMatrix eJe = robot.get_eJe("Head");
  //  std::cout << "Jacobian eJe of the " << names << ": "<< std::endl << eJe << std::endl;



  //  {
  //    // Test with a chain of joints
  //    std::string chain = "Head";

  //    std::cout << "Test " << chain << " velocity control" << std::endl;

  //    robot.setStiffness(chain, 1.f);

  //    vpColVector jointVel( robot.getBodyNames( chain ).size() );
  //    for (unsigned int i=0; i < jointVel.size(); i++)
  //      jointVel[i] = vpMath::rad(-2);

  //    double t_initial = vpTime::measureTimeSecond();
  //    while (vpTime::measureTimeSecond() < t_initial+3)
  //    {
  //      robot.setVelocity(chain, jointVel,true);
  //    }

  //    t_initial = vpTime::measureTimeSecond();
  //    while (vpTime::measureTimeSecond() < t_initial+3)
  //    {
  //      robot.setVelocity_eachJoint(chain, jointVel,true);
  //    }


  //    robot.stop(chain);
  //  }




  //  return 0;





  //    if (robot.getRobotType() == vpNaoqiRobot::Romeo) {


  //      {
  //        // Test with a chain of joints
  //        std::string chain = "Head";

  //        std::cout << "Test " << chain << " velocity control" << std::endl;

  //        robot.setStiffness(chain, 1.f);

  //        vpColVector jointVel( robot.getBodyNames( chain ).size() );
  //        for (unsigned int i=0; i < jointVel.size(); i++)
  //          jointVel[i] = vpMath::rad(-2);

  //        double t_initial = vpTime::measureTimeSecond();
  //        while (vpTime::measureTimeSecond() < t_initial+3)
  //        {
  //          robot.setVelocity(chain, jointVel);
  //        }

  //        robot.stop(chain);
  //      }


  //      {
  //        // Get the position of the joints
  //        std::string names = "Body";
  //        std::cout << "Test " << names << " retrieving joint positions" << std::endl;

  //        vpColVector jointPositions = robot.getPosition(names);
  //        std::vector<std::string> jointNames = robot.getBodyNames(names);
  //        for (unsigned int i=0; i< jointPositions.size(); i++)
  //          std::cout << "Sensor " << jointNames[i] << " position: " << jointPositions[i] << std::endl;
  //      }


  //      {
  //        //Get Transformation matrix between Frame HeadRoll and CameraLeft

  //        vpHomogeneousMatrix cMe = robot.get_cMe("CameraLeft");
  //        std::cout << "Transformation matrix between Frame HeadRoll and CameraLeft is : "<< cMe << std::endl;
  //      }

  //      std::cout << "The end" << std::endl;
  //    }
  //  }
  //  catch (const vpException &e)
  //  {
  //    std::cerr << "Caught exception: " << e.what() << std::endl;
  //  }
  //  catch (const AL::ALError &e)
  //  {
  //    std::cerr << "Caught exception: " << e.what() << std::endl;
  //  }

  return 0;
}

