
#include "visp_naoqi/common/vpFaceTrackerOkao.h"
#include "al/alvisiondefinitions.h"

vpFaceTrackerOkao::vpFaceTrackerOkao(const qi::SessionPtr &session) :m_pMemory(session->service("ALMemory")),m_pFaceDetection(session->service("ALFaceDetection")),
  m_scores(), m_image_height(240), m_image_width(320)

{

  // Start the face recognition engine
  const int period = 50;
  m_pFaceDetection.call<void>("subscribe", "Face", period, 0.0 );
  m_pFaceDetection.call<bool>("setResolution", AL::kQVGA );
  m_pFaceDetection.call<void>("enableTracking", true );
  m_pFaceDetection.call<void>("enableRecognition", true );

  m_previuos_cog.set_uv(m_image_width / 2, m_image_height/2);

}

vpFaceTrackerOkao::~vpFaceTrackerOkao()
{
  m_pFaceDetection.call<void>("unsubscribe", "Face");
}

/*!
   Allows to detect a face in the image. When more than one face is detected, faces are sorted from largest to smallest.

   \return true if one or more faces are found, false otherwise.

   The number of detected faces is returned using getNbObjects().
   If a face is found the functions getBBox(), getCog() return some information about the location of the face.

   The largest face is always available using getBBox(0) or getCog(0).
 */

bool vpFaceTrackerOkao::detect()
{
  m_message.clear();
  m_polygon.clear();
  m_nb_objects = 0;
  m_faces.clear();
  m_scores.clear();

  bool target_found = false;
  qi::AnyValue data = m_pMemory.call<qi::AnyValue>("getData", "FaceDetected");
  qi::AnyReferenceVector result;
  std::ostringstream ss;
  try
  {
    result = data.asListValuePtr();
  }
  catch(std::runtime_error& e)
  {
    ss << "Could not transform AnyValue into list: " << e.what();
    throw std::runtime_error(ss.str());
  }

  //-- Detect faces

  if (result.size() >=2)
  {
    std::cout << "face detected" << std::endl;
    qi::AnyReferenceVector info_face_array = result[1].asListValuePtr();
    std::cout << "info_face_array.size()" <<info_face_array.size() << std::endl;
    target_found = true;
    double min_dist = m_image_width*m_image_height;
    unsigned int index_closest_cog = 0;
    vpImagePoint closest_cog;
    for (unsigned int i = 0; i < info_face_array.size()-1; i++ )
    {
      //Extract face info
      // Face Detected [1]/ First face [0]/ Shape Info [0]/ Alpha [1]
      qi::AnyReference ref1 = info_face_array[i].content(); // FaceDetected[i]
      qi::AnyReference ref2 = ref1[0].content(); // ShapeInfo
      float alpha = ref2[1].content().asFloat();
      float beta = ref2[2].content().asFloat();
      float sx = ref2[3].content().asFloat();
      float sy = ref2[4].content().asFloat();
      qi::AnyReference ref3 = ref1[1].content(); // ExtraInfo
      std::string name = ref3[2].content().asString();
      float score = ref3[1].content().asFloat();

      std::ostringstream message;
      if (score > 0.6)
        message << name;
      else
        message << "Unknown";

      m_message.push_back( message.str() );
      m_scores.push_back(score);
      // sizeX / sizeY are the face size in relation to the image
      float h = m_image_height * sx;
      float w = m_image_width * sy;

      // Center of face into the image
      float x = m_image_width / 2 - m_image_width * alpha;
      float y = m_image_height / 2 + m_image_height * beta;

      vpImagePoint cog(x,y);
      double dist = vpImagePoint::distance(m_previuos_cog,cog);

      if (dist< min_dist)
      {
        closest_cog = cog;
        index_closest_cog = i;
        min_dist = dist;
      }

      std::vector<vpImagePoint> polygon;
      double x_corner = x - h/2;
      double y_corner = y - w/2;

      polygon.push_back(vpImagePoint(y_corner  , x_corner  ));
      polygon.push_back(vpImagePoint(y_corner+w, x_corner  ));
      polygon.push_back(vpImagePoint(y_corner+w, x_corner+h));
      polygon.push_back(vpImagePoint(y_corner  , x_corner+h));

      m_polygon.push_back(polygon);
      m_nb_objects ++;
    }

    if (index_closest_cog !=0)
      std::swap(m_polygon[0], m_polygon[index_closest_cog]);
    m_previuos_cog = closest_cog;
  }

  return target_found;
}


bool vpFaceTrackerOkao::detect(const vpImage<unsigned char> &I)
{
  return detect();
}

float vpFaceTrackerOkao::getScore(unsigned int i) const
{
  return m_scores[i];
}

/*!
   Remove all learned faces from the database.

   \return true if the operation succeeded
 */
bool vpFaceTrackerOkao::clearDatabase()
{
  return m_pFaceDetection.call<bool>("clearDatabase");
}

/*!
   Delete from the database all learned faces corresponding to the specified person.

  \param:	name – The name of the person to forget
  \return: true if the operation succeeded

 */
bool vpFaceTrackerOkao::forgetPerson(const std::string& name)
{
  return m_pFaceDetection.call<bool>("forgetPerson", name);
}

