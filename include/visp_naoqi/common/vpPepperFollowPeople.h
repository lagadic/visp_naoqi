#ifndef __vpPepperFollowPeople_h__
#define __vpPepperFollowPeople_h__

//#include <alproxies/almemoryproxy.h>
//#include <alproxies/alfacedetectionproxy.h>
//#include <alproxies/alpeopleperceptionproxy.h>
//#include <alproxies/alspeechrecognitionproxy.h>
//#include <alproxies/alledsproxy.h>
//#include <alproxies/altexttospeechproxy.h>


#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorBase.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpImagePoint.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPlot.h>


//#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/highgui/highgui.hpp>


#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

#include <visp_naoqi/common/vpFaceTrackerOkao.h>

#include <visp/vpDisplayX.h>


class vpPepperFollowPeople
{
public:
  typedef enum {
    state_base_follow, //!< Move base in translation and rotation
    state_base_rotate,   //!< Move base in rotation only
    state_finish        //!< Ends the demo
  } state_t;

protected:

  // Robot
  vpCameraParameters m_cam;
  vpHomogeneousMatrix m_eMc;
  vpNaoqiRobot * m_robot;
  std::vector<std::string> m_jointNames_head;
  std::vector<std::string> m_vocabulary;
  std::string m_phrase;
  //AL::ALValue m_limit_yaw;

  // Proxies and tracker
  //AL::ALMemoryProxy m_mem_proxy;
  qi::AnyObject m_pMemory; //!< Memory proxy
  //AL::ALPeoplePerceptionProxy m_people_proxy;
  qi::AnyObject m_pPeoplePerception; //!< PeoplePerception proxy
  //AL::ALSpeechRecognitionProxy * m_asr_proxy;
  qi::AnyObject * m_pSpeechRecognition; //!< ALSpeechRecognition proxy
  qi::AnyObject m_pLeds; //!< ALLeds proxy
  //AL::ALTextToSpeechProxy m_tts_proxy;
  qi::AnyObject m_pTextToSpeech; //!< TextToSpeech proxy

  //AL::ALLedsProxy m_led_proxy;
  vpFaceTrackerOkao m_face_tracker;

  // Servo
  vpServo m_task;
  vpAdaptiveGain m_lambda_base_follow;
  vpAdaptiveGain m_lambda_base_rotate;
 // AL::ALFaceDetectionProxy m_proxy;
  std::vector<cv::Rect> m_faces;  //!< Bounding box of each detected face.
  std::vector<float> m_scores;
  const int m_image_height;
  const int m_image_width;
  vpImagePoint m_previuos_cog;
  double m_Zd;
  double m_Z;
  double m_servo_time_init;
  vpColVector m_q_dot;
  double m_t_prev;
  vpImagePoint m_head_cog_cur;
  vpImagePoint m_ip;
  vpFeatureDepth m_s_Z;
  vpFeatureDepth m_s_Zd;
  vpFeaturePoint m_s;
  vpFeaturePoint m_sd;


  vpImage<unsigned char> I;
  vpDisplayX d;

  vpMatrix tJe;
  vpMatrix eJe;

  //Bool
  bool m_stop_vxy;
  state_t m_state;
  state_t m_state_prev;
  bool m_reinit_servo;
  bool m_person_found;
  bool m_person_or_face_detected;
  bool m_reverse;

  std::string m_language;

public:
  vpPepperFollowPeople(const qi::SessionPtr &session, vpNaoqiRobot &robot, const std::string &language="English");
  vpPepperFollowPeople(const qi::SessionPtr &session, vpNaoqiRobot *robot, const std::string &language="English");
  vpPepperFollowPeople(const qi::SessionPtr &session, vpNaoqiRobot *robot, qi::AnyObject * asr_proxy, const std::vector<std::string> &vocabulary, const std::string &language="English");

  ~vpPepperFollowPeople();

  void activateTranslationBase();
  state_t computeAndApplyServo(bool apply_command = true);
  void exit();
  double getActualDistance() const;
  vpColVector getCommand() const
  { return m_q_dot;}
  bool getStatusDetection() const
  {return m_person_or_face_detected;}
  void initialization();
  void setLambdaBaseFollow(vpAdaptiveGain &gain)
  { m_lambda_base_follow = gain;}
  void setDesiredDistance(double dist);
  void setReverse(bool flag);

  void stopTranslationBase();
  void stop();


};

#endif

