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
#include <visp/vpPlot.h>

#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>

/*!

   Connect toRomeo robot, and apply some motion.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./motion -ip <robot ip address>

   Example:

   ./motion -ip 169.254.168.230
 */


vpColVector computeSecondaryTaskManipulability(const vpMatrix &P, vpMatrix &J, std::vector <vpMatrix> &dJ,double & cond)
{
    const unsigned int n = dJ.size();

    vpColVector z(n);

    double alpha = 10;

//    vpMatrix v;
//    vpColVector w;
//    J.svd(w, v);
//    //std::cout << "singular values:/n" << w << std::endl;
//    cond = w[0]/w[5];

//    double sqtr_detJJt = 1.0;

//    for (unsigned int i = 0; i < n-1; i++)
//        sqtr_detJJt *= w[i];
//    std::cout << "sqtr_detJJt:/n" << sqtr_detJJt << std::endl;


//    std::cout << "detJJt" << detJJt << std::endl;
    double detJJt = (J * J.transpose()).det();
     std::cout << "sqtr_detJJMulti" << sqrt(detJJt) << std::endl;

    for (unsigned int i = 0; i < n; i++)
    {
        vpMatrix dJJinv = dJ[i] * J.pseudoInverse();

        std::cout << dJ[i]  << std::endl << dJ[i] << std::endl;

        double trace = 0.0;
        for (unsigned int k = 0; k < dJJinv.getCols(); k++)
            trace += dJJinv[k][k];

        std::cout << "trace" << i << " " << trace << std::endl;

        z[i] = alpha * sqrt(detJJt) * trace;


    }

    return z;

}




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

        vpImage<unsigned char> I(320, 320);
        vpDisplayX d(I);
        vpDisplay::setTitle(I, "ViSP viewer");

        robot.open();
        double cond = 0.0;


        vpPlot *plotter_cond;
        if (1) {
            plotter_cond = new vpPlot(1, 640, 640, 90, 90, "Loop time");
            plotter_cond->initGraph(0, 1);
        }


        const std::string chainName = "LArm";

        std::vector<std::string> jointNames = robot.getBodyNames(chainName);
        jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo


        unsigned int numJoint = jointNames.size();
        double t_initial = vpTime::measureTimeSecond();
        while (1)
        {
            //** Set task eJe matrix
            // Get the actual Jacobian of the Larm
            vpMatrix tJe;
            vpMatrix eJe = robot.get_eJe(chainName, tJe);


            // std::cout << "Jacobian of the LArm: "<< std::endl << eJe_LArm << std::endl;
            //oJo = oVe_LArm * eJe_LArm;
            //std::cout << "Jacobian of the object: "<< std::endl << eJe_LArm << std::endl;
            vpDisplay::display(I);

            std::vector <vpMatrix> dJ = robot.get_d_eJe(chainName);

            vpMatrix P (numJoint,numJoint);

            vpColVector q_man = computeSecondaryTaskManipulability(P,tJe,dJ,cond);
            std::cout << "q_man:" << q_man << std::endl;

//            vpMatrix v;
//            vpColVector w;

//            tJe.svd(w, v);
//            //std::cout << "singular values:/n" << w << std::endl;
//            cond = w[0]/w[5];

            std::cout << "Cond:" << cond << std::endl;
            plotter_cond->plot(0, 0, vpTime::measureTimeSecond()-t_initial, cond);


           // robot.setVelocity(jointNames, q_man);
            if (vpDisplay::getClick(I, false))
              break;
        }

        robot.stop(chainName);
        delete plotter_cond;







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

