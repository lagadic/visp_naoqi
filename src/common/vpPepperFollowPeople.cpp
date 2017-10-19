
#include <visp/vpImageConvert.h>
#include "visp_naoqi/common/vpPepperFollowPeople.h"

#include "al/alvisiondefinitions.h"


vpPepperFollowPeople::vpPepperFollowPeople(const qi::SessionPtr &session, vpNaoqiRobot &robot, const std::string &language)
  : m_pMemory(session->service("ALMemory")), m_pPeoplePerception(session->service("ALPeoplePerception")),
    m_pLeds(session->service("ALLeds")), m_pTextToSpeech(session->service("ALTextToSpeech")), m_face_tracker(session), m_robot(&robot),
    m_image_height(240), m_image_width(320), m_language(language)

{
  m_pSpeechRecognition = new qi::AnyObject(session->service("ALSpeechRecognition"));
  //m_asr_proxy = new  AL::ALSpeechRecognitionProxy(ip, port);
  if (language == "French") {
    m_vocabulary.push_back("suis moi");
    m_vocabulary.push_back("stop");
    m_vocabulary.push_back("quitte");
  }
  else {
    m_vocabulary.push_back("follow me");
    m_vocabulary.push_back("stop");
    m_vocabulary.push_back("close");
  }

  initialization();
}


vpPepperFollowPeople::vpPepperFollowPeople(const qi::SessionPtr &session, vpNaoqiRobot * robot, const std::string &language)
  : m_pMemory(session->service("ALMemory")), m_pPeoplePerception(session->service("ALPeoplePerception")),
    m_pLeds(session->service("ALLeds")), m_pTextToSpeech(session->service("ALTextToSpeech")), m_face_tracker(session), m_robot(robot),
    m_image_height(240), m_image_width(320), m_language(language)

{
  m_pSpeechRecognition = new qi::AnyObject(session->service("ALSpeechRecognition"));

  if (language == "French") {
    m_vocabulary.push_back("suis moi");
    m_vocabulary.push_back("stop");
    m_vocabulary.push_back("quitte");
  }
  else {
    m_vocabulary.push_back("follow me");
    m_vocabulary.push_back("stop");
    m_vocabulary.push_back("close");
  }

  initialization();
}


vpPepperFollowPeople::vpPepperFollowPeople(const qi::SessionPtr &session, vpNaoqiRobot * robot, qi::AnyObject * asr_proxy, const std::vector<std::string> &vocabulary, const std::string &language)
  : m_pMemory(session->service("ALMemory")), m_pPeoplePerception(session->service("ALPeoplePerception")),
    m_pLeds(session->service("ALLeds")), m_pTextToSpeech(session->service("ALTextToSpeech")), m_face_tracker(session),
    m_robot(robot),  m_image_height(240), m_image_width(320), m_language(language)
{
  m_pSpeechRecognition = asr_proxy;

  m_vocabulary.insert( vocabulary.end(), vocabulary.begin(), vocabulary.end() );

  initialization();
}


void vpPepperFollowPeople::initialization()
{
  // Get the camera parameters
  std::string camera_name = "CameraTopPepper";
  m_cam = vpNaoqiGrabber::getIntrinsicCameraParameters(AL::kQVGA, camera_name, vpCameraParameters::perspectiveProjWithDistortion);
  m_eMc = vpNaoqiGrabber::getExtrinsicCameraParameters(camera_name, vpCameraParameters::perspectiveProjWithDistortion);

  std::cout << "eMc:" << std::endl << m_eMc << std::endl;
  std::cout << "cam:" << std::endl << m_cam << std::endl;

  m_jointNames_head = m_robot->getBodyNames("Head");

  // Open Proxy for the speech
  m_pTextToSpeech.call<void>("setLanguage", m_language);
  //m_phrase = " Hi";

  // Initialize PeoplePerception
  m_pPeoplePerception.call<void>("subscribe","People", 30, 0.0);

  std::cout << "PeoplePerception started" << std::endl;

  // Open Proxy for the recognition speech
  m_pSpeechRecognition->call<void>("setLanguage", m_language);
  m_pSpeechRecognition->call<void>("pause", true);
  m_pSpeechRecognition->call<void>("setVisualExpression", false);
  //m_asr_proxy->pause(true);
  //m_asr_proxy->setVisualExpression(false);
  //m_asr_proxy->setLanguage("English");

  // Set the vocabulary
  m_pSpeechRecognition->call<void>("setVocabulary", m_vocabulary, false);

  // Start the speech recognition engine with user Test_m_asr_proxy
  m_pSpeechRecognition->call<void>("pause", false);
  m_pSpeechRecognition->call<void>("subscribe","Test_ASR");

  try
  {
    m_pMemory.call<void>("removeData", "WordRecognized");
  }
  catch (const std::exception& e) { // reference to the base of a polymorphic object
    std::cout << e.what(); // information from length_error printed
  }

  //Set bool
  m_stop_vxy = false;
  m_state = state_base_rotate;
  m_state_prev = m_state;
  m_servo_time_init = false;
  m_reinit_servo = true;
  m_person_found = false;
  m_person_or_face_detected = false;
  m_reverse = true;

  // Set Visual Servoing:
  m_task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
  m_task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
  m_lambda_base_follow.initStandard(1.0, 0.8, 7); // 2.3, 0.7, 15
  m_lambda_base_rotate.initStandard(5, 2.9, 15); // 4, 0.5, 15

  // Create the desired visual feature
  m_head_cog_cur.set_uv(m_image_width/2, m_image_height/2);
  m_ip.set_uv(m_image_width/2, m_image_height/2);
  // Create the current x visual feature
  m_ip.set_uv(m_image_width/2,m_image_height/2);

  vpFeatureBuilder::create(m_s, m_cam, m_ip);
  vpFeatureBuilder::create(m_sd, m_cam, m_ip);
  // Add the feature
  m_task.addFeature(m_s, m_sd);


  // Create the depth feature
  m_Zd = 1.2;
  m_Z = m_Zd;
  m_s_Z.buildFrom(m_s.get_x(), m_s.get_y(), m_Z , 0.); // log(Z/Z*) = 0 that's why the last parameter is 0
  m_s_Zd.buildFrom(m_sd.get_x(), m_sd.get_y(), m_Zd , 0.); // log(Z/Z*) = 0 that's why the last parameter is 0
  // Add the feature
  m_task.addFeature(m_s_Z, m_s_Zd);

  // Initialization Jacobian
  tJe.resize(6,5,true);
  tJe[0][0]= 1;
  tJe[1][1]= 1;
  tJe[5][2]= 1;
  eJe.resize(6,5,true);

  // m_limit_yaw = m_robot->getProxy()->getLimits("HeadYaw");

  m_t_prev = vpTime::measureTimeSecond();
  m_q_dot.resize(6,true);

  // m_robot->getProxy()->setExternalCollisionProtectionEnabled("Move", false);



}


vpPepperFollowPeople::~vpPepperFollowPeople()
{
  m_pTextToSpeech.call<void>("setLanguage", "French");
  //m_tts_proxy.setLanguage("French");

  //m_pTextToSpeech.call<void>("setLanguage", "English");
  m_pPeoplePerception.call<void>("unsubscribe", "People");

  m_pSpeechRecognition->call<void>("unsubscribe", "Test_ASR");
  //m_asr_proxy->unsubscribe("Test_ASR");
  m_pSpeechRecognition->call<void>("removeAllContext");
  // m_asr_proxy->removeAllContext();
  m_pSpeechRecognition->call<void>("setVisualExpression", true);
  //m_asr_proxy->setVisualExpression(true);
  m_pSpeechRecognition->call<void>("setLanguage", "French");


  // m_robot->getProxy()->setExternalCollisionProtectionEnabled("Move", true);
  // m_robot = NULL;
}


vpPepperFollowPeople::state_t vpPepperFollowPeople::computeAndApplyServo(bool apply_command)
{
  if (m_reinit_servo) {
    m_servo_time_init = vpTime::measureTimeSecond();
    m_t_prev = vpTime::measureTimeSecond();
    m_reinit_servo = false;
    m_pLeds.async<void>("fadeRGB", "FaceLeds", "white", 0.1);
  }

  m_stop_vxy = false;

  qi::AnyValue data_word_recognized;
  qi::AnyReferenceVector result_speech;

  try{
    data_word_recognized =  m_pMemory.call<qi::AnyValue>("getData", "WordRecognized");
    result_speech = data_word_recognized.asListValuePtr();
  }
  catch(const std::exception& e) { // reference to the base of a polymorphic object
    std::cout << e.what(); // information from length_error printed
  }
  if ( !result_speech.empty() )
  {
    // move base
    if ( ((result_speech[0].content().toString()) == m_vocabulary[0]) && (result_speech[1].content().toFloat() > 0.4 )) //move
    {
      std::cout << "Recognized: " << result_speech[0].content().toString() << "with confidence of " << result_speech[1].content().toFloat()  << std::endl;
      m_task.setLambda(m_lambda_base_follow) ;
      m_state = state_base_follow;
    }
    // stop base motion
    else if ( (result_speech[0].content().toString() == m_vocabulary[1]) && (result_speech[1].content().toFloat() > 0.4 )) //stop
    {
      std::cout << "Recognized: " << result_speech[0].content().toString() << "with confidence of " << result_speech[1].content().toFloat() << std::endl;
      m_task.setLambda(m_lambda_base_rotate) ;
      m_state = state_base_rotate;
    }
    else if ( (result_speech[0].content().toString() == m_vocabulary[2]) && (result_speech[1].content().toFloat() > 0.4 )) //stop
    {
      std::cout << "Recognized: " << result_speech[0].content().toString() << "with confidence of " << result_speech[1].content().toFloat() << std::endl;
      m_task.setLambda(m_lambda_base_rotate) ;
      m_state = state_finish;
    }
  }

  if (m_state != m_state_prev)
  {
    if (m_language == "French") {
      if (m_state == state_base_follow)
        m_pTextToSpeech.async<void>("say", "Ok, Je vais te suivre.");
      else if (m_state == state_base_rotate)
        m_pTextToSpeech.async<void>("say", "Ok, J'arrete de te suivre.");
      else if (m_state == state_finish)
        m_pTextToSpeech.async<void>("say", "Ok, Je vais quitter l'application.");
    }
    else {
      if (m_state == state_base_follow)
        m_pTextToSpeech.async<void>("say", "Ok, I will follow you.");
      else if (m_state == state_base_rotate)
        m_pTextToSpeech.async<void>("say", "Ok, I will stop following you.");
      else if (m_state == state_finish)
        m_pTextToSpeech.async<void>("say", "Ok, I will close this application.");
    }
  }

  m_state_prev = m_state;

  // Detect Face from Okao
  bool face_found = m_face_tracker.detect();
  // std::cout << "Loop time face_tracker: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
  if (face_found) {
    m_pLeds.async<void>("fadeRGB", "FaceLeds", "blue", 0.1);

    double u = m_face_tracker.getCog(0).get_u();
    double v = m_face_tracker.getCog(0).get_v();
    if (u <= m_image_width && v <= m_image_height)
      m_head_cog_cur.set_uv(u,v);
  }

  // Detect Person from Depth Camera
  qi::AnyValue data_people = m_pMemory.call<qi::AnyValue>("getData", "PeoplePerception/VisiblePeopleList");
  qi::AnyReferenceVector data_people_ref = data_people.asListValuePtr() ;

  m_person_found = false;
  if (data_people_ref.size() > 0)
  {
    qi::AnyValue info = m_pMemory.call<qi::AnyValue>("getData", "PeoplePerception/PeopleDetected");
    qi::AnyReferenceVector result = info.asListValuePtr();
    qi::AnyReferenceVector person_datas = result[1].asListValuePtr();

    int num_people = person_datas.size();
    std::ostringstream text;
    text << "Found " << num_people << " person(s)";

    m_person_found = true;

    if (face_found) // Try to find the match between two detection
    {
      vpImagePoint cog_face;
      double dist_min = 1000;
      unsigned int index_person = 0;

      for (unsigned int i = 0; i < num_people; i++)
      {
        qi::AnyReference ref1 = person_datas[i].content(); // PersonData_i[i]
        float alpha = ref1[2].content().asFloat();
        float beta = ref1[3].content().asFloat();
        //Z = Zd;
        // Centre of face into the image
        float x =  m_image_width/2 -  m_image_width * beta;
        float y =  m_image_height/2  + m_image_height * alpha;
        cog_face.set_uv(x,y);
        double dist = vpImagePoint::distance(cog_face, m_head_cog_cur);

        if (dist < dist_min)
        {
          dist_min = dist;
          //best_cog_face_peoplep = cog_face;
          index_person  = i;
        }
      }

      if (dist_min < 55.)
      {
        qi::AnyReference ref_person = person_datas[index_person].content();
        m_Z = ref_person[1].content().asFloat(); // Current distance
      }

    }
    else // Take the first one on the list and use cog face from PeoplePerception
    {
      qi::AnyReference ref1 = person_datas[0].content(); // PersonData_i[i]
      float alpha = ref1[2].content().asFloat();
      float beta = ref1[3].content().asFloat();
      //Z = Zd;
      // Centre of face into the image
      float x =  m_image_width/2 -  m_image_width * beta;
      float y =  m_image_height/2  + m_image_height * alpha;
      m_head_cog_cur.set_uv(x,y);
      m_Z = ref1[1].content().asFloat();
    }
  }
  else {
    std::cout << "No distance computed " << std::endl;
    m_stop_vxy = true;
  }

  //std::cout << "Loop time before VS: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

  m_person_or_face_detected = false;

  if (face_found || m_person_found )
  {
    // Get Head Jacobian (6x2)
    vpMatrix torso_eJe_head;
    m_robot->get_eJe("Head",torso_eJe_head);

    // Add column relative to the base rotation (Wz)
    for (unsigned int i = 0; i < 6; i++)
      for (unsigned int j = 0; j < torso_eJe_head.getCols(); j++)
        tJe[i][j+3] = torso_eJe_head[i][j];

    vpHomogeneousMatrix torsoMHeadPith = m_robot->getTransform("HeadPitch", 0);// get transformation  matrix between torso and HeadRoll
    vpVelocityTwistMatrix HeadPitchVLtorso(torsoMHeadPith.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        HeadPitchVLtorso[i][j+3] = 0;

    eJe = HeadPitchVLtorso *tJe;

    m_task.set_eJe( eJe );
    m_task.set_cVe( vpVelocityTwistMatrix(m_eMc.inverse()) );

    //  std::cout << "head_cog_des:" << std::endl << head_cog_des << std::endl;
    //  std::cout << "head_cog_cur:" << std::endl << head_cog_cur << std::endl;

    if (m_reverse == false && (m_Z - m_Zd) < 0.0 )
      m_Z = m_Zd;

    double x,y;
    vpPixelMeterConversion::convertPoint(m_cam, m_head_cog_cur, x, y);
    m_s.buildFrom(x, y, m_Z);
    //s.set_xyZ(head_cog_cur.get_u(), head_cog_cur.get_v(), Z);

    // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
    m_s_Z.buildFrom(m_s.get_x(), m_s.get_y(), m_Z, log(m_Z/m_Zd)) ;

    m_q_dot = m_task.computeControlLaw(vpTime::measureTimeSecond() - m_servo_time_init);

    //std::cout << "Loop time compute VS: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

    vpMatrix P = m_task.getI_WpW();
    double alpha = -3.3;
    double min = m_robot->getJointMin("HeadYaw")[0];
    double max = m_robot->getJointMax("HeadYaw")[0];
    vpColVector z_q2 (m_q_dot.size());
    vpColVector q_yaw = m_robot->getPosition(m_jointNames_head[0]);

    z_q2[3] = 2 * alpha * q_yaw[0]/ pow((max - min),2);

    vpColVector q3 = P * z_q2;
    //m_q_dot =  m_q_dot + q3; HACK for DEMO

    std::vector<float> vel(m_jointNames_head.size());
    vel[0] = m_q_dot[3];
    vel[1] = m_q_dot[4];

    m_person_or_face_detected = true;

    if (apply_command)
    {
      m_robot->setVelocity(m_jointNames_head, vel);

      std::cout << "q:" << m_q_dot << std::endl;

      if ( (std::fabs(m_Z - m_Zd) < 0.05) || m_stop_vxy || (m_state == state_base_rotate) )
      {
        //      std::cout << "#################################################!" << std::endl;
        //      std::cout << std::fabs(m_Z - m_Zd) << std::endl;
        //      std::cout << m_stop_vxy << std::endl;
        //      std::cout << !m_move_base << std::endl;
        //      std::cout << "#################################################!" << std::endl;
        m_robot->setBaseVelocity(0.0, 0.0, m_q_dot[2]);
      }
      else if(m_state == state_base_follow) {
        m_robot->setBaseVelocity(m_q_dot[0], m_q_dot[1], m_q_dot[2]);
      }

    }
  }
  else {
    m_robot->stop(m_jointNames_head);
    m_robot->stopBase();
    std::cout << "Stop******************************!" << std::endl;
    m_reinit_servo = true;
  }

  return m_state;
}

void  vpPepperFollowPeople::setDesiredDistance(double dist)
{
  m_Zd = dist;
  m_s_Zd.buildFrom(m_sd.get_x(), m_sd.get_y(), m_Zd , 0.); // log(Z/Z*) = 0 that's why the last parameter is 0
}

void vpPepperFollowPeople::setReverse(bool flag)
{
  m_reverse = flag;
}

double vpPepperFollowPeople::getActualDistance() const
{
  return m_Z;
}

void vpPepperFollowPeople::stop()
{
  m_robot->stop(m_jointNames_head);
  m_robot->stopBase();
  std::cout << "Stop request!" << std::endl;
  m_reinit_servo = true;
}

void vpPepperFollowPeople::stopTranslationBase()
{
  m_state = state_base_rotate;
}

void vpPepperFollowPeople::activateTranslationBase()
{
  m_state = state_base_follow;
}

void vpPepperFollowPeople::exit()
{
  m_state = state_finish;;
}




