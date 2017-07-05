/**
 *
 * This example demonstrates how to get images from the robot remotely, how
 * to track a face using all the four joints of the Romeo Head;
 *
 */

/*! \example servo_face_detection_visp_head.cpp */

#include <iostream>
#include <string>
#include <map>


#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>

#include <al/alvisiondefinitions.h>


#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ViSP includes.
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpImagePoint.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp_naoqi/common/vpServoHead.h>
#include <visp_naoqi/common/vpFaceTrackerOkao.h>

/*!

  Connect to Pepper robot, grab, display images using ViSP and start
  face detection with Okao .
  More over all the two joints of Pepper's head are controlled by visual servoing to center
  the detected head in the image.
  By default, this example connect to a robot with ip address: 131.254.10.126.
  If you want to connect on an other robot, run:

  ./servo_face_detection_visp_head_okao_pepper --ip <robot ip address>

  Example:

  ./servo_face_detection_visp_head_okao_pepper --ip 169.254.168.230
 */



bool in_array(const std::string &value, const std::vector<std::string> &array)
{
  return std::find(array.begin(), array.end(), value) != array.end();
}

bool pred(const std::pair<std::string, int>& lhs, const std::pair<std::string, int>& rhs)
{
  return lhs.second < rhs.second;
}



int main(int argc, const char* argv[])
{
  std::string opt_ip = "192.168.0.24";
  bool opt_language_english = true;
  int opt_cam = 0;

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--fr")
      opt_language_english = false;
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << " [--ip <robot address>] [--fr] [--help]" << std::endl;
      return 0;
    }
  }

  // Connection to module to control Pepper in velocity
  qi::SessionPtr session = qi::makeSession();
  std::string ip_port = "tcp://" + opt_ip + ":9559";
  session->connect(ip_port);
  if (! opt_ip.empty()) {
    std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
  }

  // Open the grabber for the acquisition of the images from the robot
  vpNaoqiGrabber g(session);
  g.setCamera(opt_cam); // Select camera
  g.setCameraResolution(AL::kQVGA);
  g.open();

  vpImage<unsigned char> I;
  g.acquire(I);

  std::cout << "Image: " << I.getHeight() <<" x " << I.getWidth() << std::endl;

  std::string camera_name = "CameraTopPepper";
  vpCameraParameters cam = vpNaoqiGrabber::getIntrinsicCameraParameters(AL::kQVGA, camera_name, vpCameraParameters::perspectiveProjWithDistortion);
  vpHomogeneousMatrix eMc = vpNaoqiGrabber::getExtrinsicCameraParameters(camera_name, vpCameraParameters::perspectiveProjWithDistortion);

  std::cout << "eMc:" << std::endl << eMc << std::endl;
  std::cout << "cam:" << std::endl << cam << std::endl;

  // Connect to the robot
  vpNaoqiRobot robot(session);
  robot.open();

  if (robot.getRobotType() != vpNaoqiRobot::Pepper) {
    std::cout << "ERROR: You are not connected to Pepper, but to a different Robot. Check the IP. " << std::endl;
    return 0;
  }

  std::vector<std::string> jointNames_head = robot.getBodyNames("Head");

  // Open Proxy for the speech
  qi::AnyObject pTextToSpeech(session->service("ALTextToSpeech"));

  std::string phraseToSay;
  if (opt_language_english) {
    pTextToSpeech.call<void>("setLanguage", "English");
    phraseToSay = " \\emph=2\\ Hi,\\pau=200\\ How are you ?";
  }
  else {
    pTextToSpeech.call<void>("setLanguage", "French");
    phraseToSay = " \\emph=2\\ Bonjour,\\pau=200\\ comment vas  tu ?";
  }

  pTextToSpeech.async<void>("say", phraseToSay);

  try {
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    vpFaceTrackerOkao face_tracker(session);

    // Initialize head servoing
    vpServoHead servo_head;
    servo_head.setCameraParameters(cam);
    vpAdaptiveGain lambda(3.5, 0.5, 15); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
    servo_head.setLambda(lambda);

    double servo_time_init = 0;

    vpImagePoint head_cog_cur;
    vpImagePoint head_cog_des(I.getHeight()/2, I.getWidth()/2);

    bool reinit_servo = true;

    std::vector<std::string> recognized_names;
    std::map<std::string,unsigned int> detected_face_map;
    bool detection_phase = true;
    unsigned int f_count = 0;

    double t_prev = vpTime::measureTimeSecond();

    while(1) {
      if (reinit_servo) {
        servo_time_init = vpTime::measureTimeSecond();
        t_prev = vpTime::measureTimeSecond();
        reinit_servo = false;
        //proxy.call<void >("start");
      }

      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);
      bool face_found = face_tracker.detect();


      if (face_found) {
        std::ostringstream text;
        text << "Found " << face_tracker.getNbObjects() << " face(s)";
        vpDisplay::displayText(I, 10, 10, text.str(), vpColor::red);
        for(size_t i=0; i < face_tracker.getNbObjects(); i++) {
          vpRect bbox = face_tracker.getBBox(i);
          if (i == 0)
            vpDisplay::displayRectangle(I, bbox, vpColor::red, false, 2);
          else
            vpDisplay::displayRectangle(I, bbox, vpColor::green, false, 1);
          vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), face_tracker.getMessage(i) , vpColor::red);
        }

        double u = face_tracker.getCog(0).get_u();
        double v = face_tracker.getCog(0).get_v();
        if (u<= g.getWidth() && v <= g.getHeight())
          head_cog_cur.set_uv(u,v);

        vpRect bbox = face_tracker.getBBox(0);
        std::string name = face_tracker.getMessage(0);


        servo_head.set_eJe( robot.get_eJe("Head") );
        servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

        servo_head.setCurrentFeature(head_cog_cur);
        servo_head.setDesiredFeature(head_cog_des);
        //vpDisplay::setFont(I, "-*-*-bold-*-*-*-*-*-*-*-*-*-*-*");
        //vpDisplay::displayText(I, face_tracker.getFace().getTopLeft()+vpImagePoint(-20,0), "Coraline", vpColor::red);
        //vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 3);

        vpColVector q_dot_head = servo_head.computeControlLaw(vpTime::measureTimeSecond() - servo_time_init);

        // Compute the distance in pixel between the target and the center of the image
        double distance = vpImagePoint::distance(head_cog_cur, head_cog_des);
        // if (distance > 0.03*I.getWidth())
        robot.setVelocity(jointNames_head, q_dot_head);

        if (detection_phase)
        {

          //if (score >= 0.4 && distance < 0.06*I.getWidth() && bbox.getSize() > 3000)
          if (distance < 0.06*I.getWidth() && bbox.getSize() > 3000)
          {
            vpDisplay::displayRectangle(I, bbox, vpColor::red, false, 1);
            vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), name, vpColor::red);
            detected_face_map[name]++;
            f_count++;
          }
          else
          {
            vpDisplay::displayRectangle(I, bbox, vpColor::green, false, 1);
            vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), name, vpColor::green);
          }
          if (f_count>10)
          {
            detection_phase = false;
            f_count = 0;
          }
        }
        else
        {
          std::string recognized_person_name = std::max_element(detected_face_map.begin(), detected_face_map.end(), pred)->first;
          unsigned int times = std::max_element(detected_face_map.begin(), detected_face_map.end(), pred)->second;

          if (!in_array(recognized_person_name, recognized_names) && recognized_person_name != "Unknown") {

            if (opt_language_english)
            {
              phraseToSay = "\\emph=2\\ Hi \\wait=200\\ \\emph=2\\" + recognized_person_name + "\\pau=200\\ How are you ?";
            }
            else
            {
              phraseToSay = "\\emph=2\\ Salut \\wait=200\\ \\emph=2\\" + recognized_person_name + "\\pau=200\\ comment vas  tu ?";;
            }

            std::cout << phraseToSay << std::endl;
            pTextToSpeech.async<void>("say", phraseToSay);
            recognized_names.push_back(recognized_person_name);
          }
          if (!in_array(recognized_person_name, recognized_names) && recognized_person_name == "Unknown"
              && times > 15)
          {

            if (opt_language_english)
            {
              phraseToSay = "\\emph=2\\ Hi \\wait=200\\ \\emph=2\\. I don't know you! \\emph=2\\ What's your name?";
            }
            else
            {
              phraseToSay = " \\emph=2\\ Salut \\wait=200\\ \\emph=2\\. Je ne te connais pas! \\emph=2\\  Comment t'appelles-tu ?";
            }

            std::cout << phraseToSay << std::endl;
            pTextToSpeech.async<void>("say", phraseToSay);
            recognized_names.push_back(recognized_person_name);
          }

          detection_phase = true;
          detected_face_map.clear();

        }

      }
      else {
        robot.stop(jointNames_head);
        std::cout << "Stop!" << std::endl;
        reinit_servo = true;
      }

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    }


    robot.stop(jointNames_head);

  }
  catch(const vpException &e) {
    std::cerr << "Caught visp exception " << e.what() << std::endl;
  }

  std::cout << "The end: stop the robot..." << std::endl;
  robot.stop(jointNames_head);

  return 0;
}
