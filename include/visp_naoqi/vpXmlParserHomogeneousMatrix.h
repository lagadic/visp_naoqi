/****************************************************************************
 *
 * $Id: vpXmlParserHomogeneousMatrix.h 4649 2014-02-07 14:57:11Z fspindle $
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
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * XML parser to load and save Homogeneous Matrix in a XML file
 *
 * Authors:
 * Giovanni Claudio
 *
 *****************************************************************************/



/*!
  \file vpXmlParserHomogeneousMatrix.h
  \brief Declaration of the vpXmlParserHomogeneousMatrix class.
  Class vpXmlParserHomogeneousMatrix allowed to load and save Homogeneous Matrixes in a file XML

*/


#ifndef vpXMLPARSERHOMOGENEOUSMATRIX_H
#define vpXMLPARSERHOMOGENEOUSMATRIX_H

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <string>
#include <visp/vpXmlParser.h>
#include <visp/vpHomogeneousMatrix.h>
#include <libxml/xmlmemory.h>      /* Functions of libxml.                */

/*!
  \class vpXmlParserHomogeneousMatrix

  \ingroup CameraModelTransformation CameraModel

  \brief XML parser to load and save an Homogenous Matrix

  To have a complete description of the Homogeneous matrix implemented in ViSP, see
  vpHomogeneousMatrix.

  Example of an XML file "myXmlFile.xml" containing an Pose vector
  that will be converted in a Homogeneous Matrix:

  \code
  <?xml version="1.0"?>
<?xml version="1.0"?>
<root>
  <homogeneous_transformation>
    <!--Name of the homogeneous matrix-->
    <name>eMc_CameraRigth_without_distorsion</name>
    <values>
      <!--Translation vector-->
      <tx>1.00</tx>
      <ty>1.30</ty>
      <tz>3.50</tz>
      <!--Rotational vector expressed in angle axis representation-->
      <theta_ux>0.20</theta_ux>
      <theta_uy>0.30</theta_uy>
      <theta_uz>0.50</theta_uz>
    </values>
  </homogeneous_transformation>
</root>
  \endcode

  Example of loading an existing homogeneous matrix from an XML file:
  \code

#include <iostream>
#include <string>
#include<visp_naoqi/vpXmlParserHomogeneousMatrix.h>

int main(int argc, char* argv[])
{

    vpHomogeneousMatrix M;
#ifdef VISP_HAVE_XML2
    vpXmlParserHomogeneousMatrix p; // Create a XML parser

    std::string name =  "eMc_CameraRigth_without_distorsion";

    if (p.parse(M,"eMc.xml", name) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot found the Homogeneous matrix named " << name<< "." << std::endl;
      return 0;
    }
    else
     std::cout << "Homogeneous matrix " << name <<": " << std::endl << M << std::endl;

#endif

  return 0;
}

  \endcode

  Example of writing an homogenoeus matrix in a XML file :
  \code
#include <iostream>
#include <string>

#include<visp_naoqi/vpXmlParserHomogeneousMatrix.h>


int main(int argc, char* argv[])
{

    //Create Pose Vector and convert to Homogeneous Matrix
    vpPoseVector r(1.0,1.3,3.5,0.2,0.3,0.5);
    vpHomogeneousMatrix M(r);

#ifdef VISP_HAVE_XML2
    vpXmlParserHomogeneousMatrix p; // Create a XML parser
    std::string name_M =  "eMc_CameraLeft_with_distorsion";
    char filename[FILENAME_MAX];
    sprintf(filename, "%s", "eMc.xml");

    if (p.save(M, filename, name_M) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK)
    {
      std::cout << "Cannot save the Homogeneous matrix" << std::endl;
    }

  vpXmlParser::cleanup();

#endif
  return 0;
}
  \endcode
*/

class VISP_EXPORT vpXmlParserHomogeneousMatrix: public vpXmlParser
{

public:

  /* --- XML Code------------------------------------------------------------ */
  typedef enum
  {
    CODE_XML_BAD = -1,
    CODE_XML_OTHER,
    CODE_XML_M,
    CODE_XML_M_NAME,
    CODE_XML_VALUE,
    CODE_XML_TX,
    CODE_XML_TY,
    CODE_XML_TZ,
    CODE_XML_TUX,
    CODE_XML_TUY,
    CODE_XML_TUZ
  } vpXmlCodeType;

  typedef enum
  {
    SEQUENCE_OK    ,
    SEQUENCE_ERROR
  } vpXmlCodeSequenceType;

private :

  vpHomogeneousMatrix M;
  std::string M_name;
  double tx;
  double ty;
  double tz;
  double tux;
  double tuy;
  double tuz;


  //! Allowed size difference between input image and data from the xml parser to handle minor differences (ex. FORMAT7 can creates 648*488 images).
  static const int allowedPixelDiffOnImageSize = 15;

public:

  vpXmlParserHomogeneousMatrix();
  vpXmlParserHomogeneousMatrix(vpXmlParserHomogeneousMatrix& twinParser);
  vpXmlParserHomogeneousMatrix& operator =(const vpXmlParserHomogeneousMatrix& twinparser);
  ~vpXmlParserHomogeneousMatrix(){}

  int parse(vpHomogeneousMatrix &M_, const char * filename,
            const std::string &M_name_);

  int save(const vpHomogeneousMatrix &M, const char * filename,
           const std::string &M_name);

  // get/set functions
  std::string getHomogeneousMatrixName(){return this->M_name;}
  vpHomogeneousMatrix getHomogeneousMatrix(){return this->M;}


  void setHomogeneousMatrixName(const std::string& name){
    this->M_name = name;
  }

private:
  int read (xmlDocPtr doc, xmlNodePtr node,
            const std::string& M_name_);

  int count (xmlDocPtr doc, xmlNodePtr node,
             const std::string& M_name);

  int read_matrix (xmlDocPtr doc, xmlNodePtr node,
                   const std::string& M_name_);
  

  vpXmlCodeSequenceType read_values (xmlDocPtr doc, xmlNodePtr node,
                                           vpHomogeneousMatrix &M_tmp);
  
  static vpXmlCodeSequenceType str2xmlcode (char * str, vpXmlCodeType & res);
  void myXmlReadIntChild (xmlDocPtr doc,
                          xmlNodePtr node,
                          int &res,
                          vpXmlCodeSequenceType &code_error);

  void myXmlReadDoubleChild (xmlDocPtr doc,
                             xmlNodePtr node,
                             double &res,
                             vpXmlCodeSequenceType &code_error);

  void myXmlReadCharChild (xmlDocPtr doc,
                           xmlNodePtr node,
                           char **res);
  int write (xmlNodePtr node, const std::string& M_name);


  
private:

  /*!

    \param 2doc : a pointer representing the document
    \param node : the root node of the document
  */
  virtual void readMainClass(xmlDocPtr , xmlNodePtr ){};
  
  /*!

    
    \param node2 : the root node of the document
  */
  virtual void writeMainClass(xmlNodePtr ){};
  
};
#endif //VISP_HAVE_XML2
#endif
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

