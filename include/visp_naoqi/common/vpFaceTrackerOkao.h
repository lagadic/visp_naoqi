#ifndef __vpFaceTrackerOkao_h__
#define __vpFaceTrackerOkao_h__

// Aldebaran includes
#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>

// OpenCV
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

// Visp
#include <visp3/detection/vpDetectorBase.h>
#include <visp3/core/vpConfig.h>


class VISP_EXPORT vpFaceTrackerOkao : public vpDetectorBase
{
protected:
  qi::AnyObject m_pMemory; //!< Memory proxy
  qi::AnyObject m_pFaceDetection; //!< ALFaceDetectionProxy proxy
  std::vector<cv::Rect> m_faces;  //!< Bounding box of each detected face.
  std::vector<float> m_scores;
  const int m_image_height;
  const int m_image_width;
  vpImagePoint m_previuos_cog;



public:
  /*!
    Default destructor.
   */
  vpFaceTrackerOkao(const qi::SessionPtr &session);
  ~vpFaceTrackerOkao();

  bool clearDatabase();
  bool detect(const vpImage <unsigned char> &I);
  bool detect();
  bool forgetPerson(const std::string& name);


  float getScore(unsigned int i) const;

};

#endif

