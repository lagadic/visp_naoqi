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

#include <visp/vpMath.h>
#include <visp/vpTime.h>
#include <visp/vpColVector.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include <visp_naoqi/vpNaoqiRobot.h>


/*!

   Connect toRomeo robot, and apply some motion.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./motion -ip <robot ip address>

   Example:

   ./motion -ip 169.254.168.230
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

        vpNaoqiRobot robot;
        if (! opt_ip.empty()) {
            std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
            robot.setRobotIp(opt_ip);
        }

        robot.open();


        std::vector <vpMatrix> d_eJe =  robot.get_d_eJe("LArm");

        vpMatrix a = d_eJe.at(6) ;

        std::cout << " Return" << std::endl;
        std::cout << a << std::endl;

        return 0;





        if (robot.getRobotType() == vpNaoqiRobot::Romeo) {




            if (0){
                // Get the actual Jacobian of the Larm


                const std::string chainName = "LArm";

                std::vector<std::string> jointNames = robot.getBodyNames(chainName);
                jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo

                // Velocity end effector
                vpColVector X;
                X.resize(6);
                X[0]= 0.00;
                X[1]= -0.00;
                X[2]= 0.00;
                X[3]= 0;//vpMath::rad(3.0);
                X[4]= 0;//vpMath::rad(3.0);
                X[5]= vpMath::rad(3.0);//0.0;

                std::cout << "Test to apply a cartesian velocity to the LArm: " << X.t() << std::endl;
                vpColVector qdot;

                double t_initial = vpTime::measureTimeSecond();
                while (vpTime::measureTimeSecond() < t_initial+5)
                {
                    vpMatrix eJe = robot.get_eJe("LArm");
                    qdot = eJe.pseudoInverse() * X;
                    std::cout << "Qdot: "<< std::endl << qdot << std::endl;

                    robot.setVelocity(jointNames, qdot);
                }

                robot.stop(chainName);

                return 0;
            }


            if (0){
                // Get the actual Jacobian of the Larm
                vpMatrix eJe_Head = robot.get_eJe("Head_metapod");
                std::cout << "Jacobian METAPOD"<< std::endl << eJe_Head << std::endl;

                return 0;
            }


            {
                // Velocity end effector
                vpColVector v_o(6);
                v_o = 0;
                v_o[1] = 0.01;
                //v_o[5] = vpMath::rad(5);

                const std::string chainName = "LArm";

                std::vector<std::string> jointNames = robot.getBodyNames(chainName);
                jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo


                std::cout << "Test to apply a cartesian velocity to the object: " << v_o.t() << std::endl;
                vpColVector q_dot;

                // Constant transformation Target Frame to LArm end-effector (LWristPitch)
                vpHomogeneousMatrix oMe_LArm;


                for(unsigned int i=0; i<3; i++)
                    oMe_LArm[i][i] = 0; // remove identity
                oMe_LArm[0][2] =  1;
                oMe_LArm[1][0] = -1;
                oMe_LArm[2][1] = -1;

                oMe_LArm[0][3] = -0.022;
                oMe_LArm[1][3] =  0.035;
                oMe_LArm[2][3] = -0.045;

                std::cout <<  "oMe_LArm\n" << oMe_LArm << std::endl;


                std::string filename_transform = "/udd/gclaudio/romeo/cpp/workspace/romeo_tk/data/transformation.xml";
                std::string name_transform = "e_M_LArm_qrcode";
                {
                    vpXmlParserHomogeneousMatrix pm; // Create a XML parser

                    if (pm.parse(oMe_LArm, filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
                        std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
                        return 0;
                    }
                    else
                        std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << oMe_LArm << std::endl;
                }

                std::cout <<  "oMe_LArm transpose\n" << oMe_LArm.inverse() << std::endl;


                return 0;
                vpVelocityTwistMatrix oVe_LArm(oMe_LArm);
                vpMatrix oJo; // Jacobian in the target (=object) frame

                double t_initial = vpTime::measureTimeSecond();
                while (vpTime::measureTimeSecond() < t_initial+7)
                {
                    //** Set task eJe matrix
                    // Get the actual Jacobian of the Larm
                    vpMatrix eJe_LArm = robot.get_eJe("LArm");

                    std::cout << "Jacobian of the LArm: "<< std::endl << eJe_LArm << std::endl;

                    oJo = oVe_LArm * eJe_LArm;
                    std::cout << "Jacobian of the object: "<< std::endl << eJe_LArm << std::endl;

                    q_dot = oJo.pseudoInverse() * v_o;

                    std::cout << "q_dot: " << q_dot.t() << std::endl;

                    robot.setVelocity(jointNames, q_dot);
                }

                robot.stop(chainName);
                return 0;
            }



            {
                // Test with a vector of joints
                std::vector<std::string> jointNames;
                jointNames.push_back("NeckYaw");
                jointNames.push_back("NeckPitch");

                std::cout << "Test " << jointNames << " velocity control" << std::endl;

                vpColVector jointVel( jointNames.size() );
                for (unsigned int i=0; i < jointVel.size(); i++)
                    jointVel[i] = vpMath::rad(2);

                robot.setStiffness(jointNames, 1.f);
                double t_initial = vpTime::measureTimeSecond();
                while (vpTime::measureTimeSecond() < t_initial+3)
                {
                    robot.setVelocity(jointNames, jointVel);
                }

                robot.stop(jointNames);
            }





            {
                // Test with a chain of joints
                std::string chain = "Head";

                std::cout << "Test " << chain << " velocity control" << std::endl;

                robot.setStiffness(chain, 1.f);

                vpColVector jointVel( robot.getBodyNames( chain ).size() );
                for (unsigned int i=0; i < jointVel.size(); i++)
                    jointVel[i] = vpMath::rad(-2);

                double t_initial = vpTime::measureTimeSecond();
                while (vpTime::measureTimeSecond() < t_initial+3)
                {
                    robot.setVelocity(chain, jointVel);
                }

                robot.stop(chain);
            }


            {
                // Get the position of the joints
                std::string names = "Body";
                std::cout << "Test " << names << " retrieving joint positions" << std::endl;

                vpColVector jointPositions = robot.getPosition(names);
                std::vector<std::string> jointNames = robot.getBodyNames(names);
                for (unsigned int i=0; i< jointPositions.size(); i++)
                    std::cout << "Sensor " << jointNames[i] << " position: " << jointPositions[i] << std::endl;
            }

            {
                // Example showing how to set angles, using a fraction of max speed
                std::vector<std::string> jointNames;
                jointNames.push_back("NeckYaw");
                jointNames.push_back("NeckPitch");

                std::cout << "Test " << jointNames << " position control" << std::endl;

                std::vector<float> jointPos( jointNames.size() );
                for (unsigned int i=0; i < jointPos.size(); i++)
                    jointPos[i] = vpMath::rad(0);

                float fractionMaxSpeed  = 0.1f;
                robot.setStiffness(jointNames, 1.f);
                qi::os::sleep(1.0f);
                robot.setPosition(jointNames, jointPos, fractionMaxSpeed);
                qi::os::sleep(2.0f); // Tempo to allow to reach the position
            }




            {
                //Get Transformation matrix between Frame HeadRoll and CameraLeft

                vpHomogeneousMatrix cMe = robot.get_cMe("CameraLeft");
                std::cout << "Transformation matrix between Frame HeadRoll and CameraLeft is : "<< cMe << std::endl;
            }

            {
                //Get the actual Jacobian of the Head
                std::string names = "Head";
                std::cout << "Test " << names << " Jacobian" << std::endl;
                vpMatrix eJe = robot.get_eJe(names);
                std::cout << "Jacobian eJe of the " << names << ": "<< std::endl << eJe << std::endl;
            }

            std::cout << "The end" << std::endl;
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

