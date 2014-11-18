/****************************************************************************
 *
 * $Id: vpConfig.h.cmake,v 1.23 2008/05/16 10:02:28 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Romeo configuration.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpNaoqiConfig_h
#define vpNaoqiConfig_h

// Defined if Metapod is available
#cmakedefine VISP_NAOQI_HAVE_MATAPOD

// Where is the XML file containing the camera intrinsic parameters
#define VISP_NAOQI_INTRINSIC_CAMERA_FILE "${VISP_NAOQI_INTRINSIC_CAMERA_FILE}"

// Where is the XML file containing the camera extrinsic parameters
#define VISP_NAOQI_EXTRINSIC_CAMERA_FILE "${VISP_NAOQI_EXTRINSIC_CAMERA_FILE}"

// Where is the XML file containing general homogeneous matrix useful for the demos
#define VISP_NAOQI_GENERAL_M_FILE "${VISP_NAOQI_GENERAL_M_FILE}"


#endif


