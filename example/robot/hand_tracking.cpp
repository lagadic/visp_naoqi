/*!
 * ======================================================
 *
 *
 * DESCRIPTION: Main program demonstrating 3D tracking using 3D rendering
 *
 *
 *
 * \authors Antoine Petit
 * \date 05/06/11
 *======================================================
 */

#include <QApplication>
#include <visp3/sinatrack/scenemanager.h>
#include <visp3/sinatrack/sceneviewer.h>

#include <cstdlib>
#include <visp3/io/vpVideoReader.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/vision/vpPose.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/io/vpVideoWriter.h>

#include <fstream>
#include <math.h>
#include <string.h>
#include <highgui.h>
#include <iostream>
#include <sstream>

#include <visp3/core/vpException.h>
#include <visp3/core/vpImageException.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpImageTools.h>

#include <visp_naoqi/vpNaoqiGrabber.h>

#ifdef True
#undef True
#undef False
#endif

//#include <cv.h>
#if VISP_HAVE_OPENCV_VERSION >= 0x020101
#  include <opencv2/core/core.hpp>
#  include <opencv2/highgui/highgui.hpp>
#else
#  include <cv.h>
#  include <highgui.h>
#  include <cxcore.h>
#endif

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <visp3/sinatrack/apMbTracker.h>
#include <visp3/sinatrack/apKalmanFilter.h>
//#include "apKalmanFilterSE3.h"
//#include "apKalmanFilterSE33.h"
//#include "apKalmanFilterQuat.h"

#include <iostream>
#include <sstream>
#include <sys/stat.h>

#include <visp3/sinatrack/luaconfig.h>

#define GETOPTARGS  "p:o:i:c:s:Idhw:"

bool fileExists(const std::string& filename)
{
    struct stat buf;
    if (stat(filename.c_str(), &buf) != -1)
    {
        return true;
    }
    return false;
}

void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
          Example of tracking based on the 3D model.\n\
          \n\
          SYNOPSIS\n\
          %s [-o <object name>]\n\
      [-i <image path>] \n\
      [-s <first image>] \n\
      [-c <config file>] \n\
      [-l <learn object>] \n\
      [-m <detect object>] \n\
      [-d] [-h]",
      name );

  fprintf(stdout, "\n\
          OPTIONS:                                               \n\
          -o <object name>                                 \n\
          Specify the name of object or scene to track\n\
          \n\
          -i <input image path>                                \n\
          Set image input path.\n\
          \n\
          -s <first image>                                \n\
          Set the first image of the sequence.\n\
          \n\
          -c <config file>                                \n\
          Set the config file to use (Lua script).\n\
          \n\
          -d \n\
          Turn off the display.\n\
          \n\
          -h \n\
          Print the help.\n\n");

          if (badparam)
          fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

bool getOptions(int argc, char **argv,
                std::string &opt_path,
                std::string &object,
                std::string &ipath,
                bool &display,
                int &start_image,
                std::string &configfile,
                bool &poseInitpath,
                std::string &pathSave)
{
  const char *optarg;
  const char **argv1=(const char**)argv;
  int   c;
  while ((c = vpParseArgv::parse(argc, argv1, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'p': opt_path = optarg; break;
    case 'o': object = optarg; break;
    case 'i': ipath = optarg; break;
    case 'd': display = false; break;
    case 'c': configfile = optarg; break;
    case 's': start_image = atoi(optarg);   break;
    case 'h': usage(argv[0], NULL); return false; break;
    case 'I': poseInitpath = true; break;
    case 'w': pathSave = optarg; break;

    default:
      usage(argv[0], optarg);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}


int main(int argc, char **argv)
{
  std::cout << CV_VERSION << std::endl;

  QApplication a(argc, argv);

  // Paths and file names
  std::string opt_ipath;
  std::string ipath;
  std::string configFile;
  std::string opt_path;
  std::string opt_object;
  std::string object;
  std::string initFile;
  std::string modelFile;
  std::string gdtpath;
  std::string trueposepath;
  bool poseInitpath = false;
  std::string pathSave;
  std::vector<double> inline_init;
  bool opt_display;
  bool opt_click_allowed = true;
  int start_image = 0;

  std::string env_path_yol = "/home/Work/Data/Romeo/";

  // Read the command line options
  if (!getOptions(argc, argv, opt_path, opt_object, opt_ipath, opt_display, start_image, configFile, poseInitpath, pathSave)) {
    return (-1);
  }

  opt_display = true;
  // Get the option values
  if (!opt_path.empty())
    env_path_yol = opt_path;

  if (!opt_object.empty())
    object = opt_object;
  else
  {
    std::cout << "objectpath not specified" << std::endl;
    return 0;
  }

  if (configFile.empty())
    configFile = object + vpIoTools::path("/") + object + vpIoTools::path(".lua");

  modelFile = env_path_yol + object + vpIoTools::path("/")+ object + vpIoTools::path(".obj");
  initFile = env_path_yol + object + vpIoTools::path("/") + object;

  if(poseInitpath)
  {
    std::string posePath = env_path_yol + object + vpIoTools::path("/") + object + vpIoTools::path(".txt");
    std::cout << posePath << std::endl;
    std::ifstream file;
    file.open( posePath.c_str(), std::ios::binary );
    if( file.is_open() )
    {
      for(unsigned int i = 0 ; i < 16 ; i++)
      {
        double val;
        file >> val;
        inline_init.push_back(val);
        std::cout << inline_init[i] << std::endl;
      }
    }
  }

  apMbTracker tracker;
  // Set tracking and rendering parameters
  vpCameraParameters mcam;
  apRend mrend;

  std::cout << "Loading config file : " << configFile.c_str() << std::endl;

  tracker.loadConfigFile(configFile);
  tracker.getCameraParameters(mcam);
  tracker.getRendParameters(mrend);

  luxifer::SceneViewer viewer;
  luxifer::SceneManager *mgr = new luxifer::SceneManager(const_cast<QGLContext*>(viewer.context()));
  viewer.setSceneManager(mgr);
  viewer.show();
  viewer.move(200, 200);

  vpImage<unsigned char> Id;
  vpImage<vpRGBa> Icol, IcolWithoutFeatures;
  vpImage<vpRGBa> Ioverlay;
  vpImage<vpRGBa> Ioverlaycol, IoverlaycolWithoutFeatures;

  //VideoReader to read images from disk
//  vpVideoReader reader;
//  reader.setFileName(ipath.c_str());
//  reader.setFirstFrameIndex(start_image);
//  reader.open(Id);
//  reader.acquire(Id);

//  vpVideoReader readerRGB;
//    readerRGB.setFileName(ipath.c_str());
//    readerRGB.setFirstFrameIndex(start_image);
//    readerRGB.open(Icol);
//    readerRGB.acquire(Icol);

    vpNaoqiGrabber g;

    g.setCameraResolution(AL::kQVGA);
    g.open();
    g.setCamera(0);
    g.setFramerate(15);

    std::cout << "Image size: " << g.getWidth() << " " << g.getHeight() << std::endl;
    // Create an OpenCV image container
    cv::Mat cvI = cv::Mat(cv::Size(g.getWidth(), g.getHeight()), CV_8UC3);

    g.acquire(cvI);
    vpImageConvert::convert(cvI, Icol);
    vpImageConvert::convert(cvI, Id);


  const int width = g.getWidth();
  const int height = g.getHeight();

  viewer.resize(width, height);
  mgr->setApRend(&mrend);
  mgr->setImageSize(width, height);
  mgr->setFOV(atan(mcam.get_v0() / mcam.get_py()) * 360.0 / M_PI);
  std::cout << atan((height - mcam.get_v0()) / mcam.get_py()) * 360.0 / M_PI << std::endl;
  std::cout << atan(height * 0.5 / mcam.get_py()) * 360.0 / M_PI << std::endl;
  mgr->setAspectRatio(mcam.get_u0() * mcam.get_py() / (mcam.get_v0() * mcam.get_px()));
  mgr->load(modelFile);

  // Depth edges map, with gradient orientation
  vpImage<unsigned char> Ior(height,width);
  // Normal map and texture map
  vpImage<vpRGBa> Inormd(height,width);
  // Texture edge map
  vpImage<unsigned char> Itex(height,width);
  for (int n=0; n <height ; n++)
  {
    for (int m = 0 ; m < width; m++)
    {
      Itex[n][m]=100;
    }
  }

  // Main window creation and displaying
  vpDisplayX display1;
  vpDisplayX display2;
  vpDisplayX display3;
  if (opt_display)
  {
    display1.init(Id, 10, 10, "Test tracking");
    vpDisplay::display(Id) ;
    vpDisplay::flush(Id);
    //if(tracker.getUseRGB())
    {
      IcolWithoutFeatures = Icol;

      display2.init(Icol, 10, 1000, "Test tracking");
      vpDisplay::display(Icol) ;
      vpDisplay::flush(Icol);

      if(!pathSave.empty())
      {
        display3.init(IcolWithoutFeatures, 1000, 1000, "Test tracking without features");
        vpDisplay::display(IcolWithoutFeatures) ;
        vpDisplay::flush(IcolWithoutFeatures);
      }
    }
  }

  vpHomogeneousMatrix cMo, cMo2, cMoFilt;

  // Manual initialization of the tracker
  if (opt_display && opt_click_allowed)
  {
    if (inline_init.empty())
    {
      while(!vpDisplay::getClick(Id,false))
      {
        vpDisplay::display(Id);
        vpDisplay::displayCharString(Id, 15, 10,
                                     "click after positioning the object",
                                     vpColor::red);
        vpDisplay::flush(Id) ;
      }
      tracker.initClick(Id, initFile.c_str(), true);
    }
    else
    {
      std::cout << "Inline init" << std::endl;
      memcpy(cMo.data, inline_init.data(), sizeof(double) * inline_init.size());
      std::cout << cMo << std::endl;
      tracker.initFromPose(Id, cMo);
    }
  }

  tracker.setIprec(Id);
  tracker.cMoprec = cMo;
  tracker.setIprecRGB(Icol);
  tracker.getPose(cMo);
  //tracker.setGroundTruth(gdtpath, trueposepath, start_image);
  tracker.initKltTracker(Id);

  if (opt_display)
    vpDisplay::flush(Id);

  vpTranslationVector tr;
  cMo.extract(tr);

  bool useKalmanFilter = tracker.getUseKalman();
  apKalmanFilter filt;
  apKalmanParameters kparam;
  tracker.getKalmanParameters(kparam);
  vpMatrix covMat;
  vpMatrix covMatME;

  if(useKalmanFilter)
  {
    std::cout << "Using Kalman" << std::endl;
    filt.initFilter(cMo, kparam);
  }
  int im=start_image;

  tracker.setFrame(start_image);

  cMo2 = cMo;

  vpImage<double> Igrad, Igradx, Igrady;
  Igrad.resize(height,width);
  Igradx.resize(height,width);
  Igrady.resize(height,width);

  int sample = 1;
  double meantime =0;
  int meant = 1;

  //double timeKalman = 0;
  double t0,t1;

  //Results
  vpVideoWriter writerImg, writerImgWithoutFeatures;

  if(!pathSave.empty()){
  }

  int vTime = (int)vpTime::measureTimeSecond();
  std::ofstream filePoseRes;
  std::stringstream pathDirectory;

  if(!pathSave.empty())
  {
    pathDirectory << pathSave << "launch_" << vTime;

    vpIoTools::makeDirectory(pathDirectory.str());

    std::stringstream pathResultFile;
    pathResultFile << pathDirectory.str() << "/pose.dat";

    if(fileExists(pathResultFile.str()))
      throw;

    filePoseRes.open (pathResultFile.str().c_str());
    std::cout << "Saving Pose as : " << pathResultFile.str() << std::endl;

    // Saving images with features
    std::stringstream pathResultImg;
    pathResultImg << pathDirectory.str() << "/img";

    vpIoTools::makeDirectory(pathResultImg.str());

    std::stringstream pathResultImg2;
    pathResultImg2 << pathResultImg.str() << "/%04d.png";

    std::cout << "Saving Images as : " << pathResultImg2.str() << std::endl;
    writerImg.setFileName(pathResultImg2.str());
    Ioverlaycol.resize(height,width);
    writerImg.open(Ioverlaycol);

    // Saving images without features
    std::stringstream pathResultImgWF;
    pathResultImgWF << pathDirectory.str() << "/img_no_features";

    vpIoTools::makeDirectory(pathResultImgWF.str());

    std::stringstream pathResultImg2WF;
    pathResultImg2WF << pathResultImgWF.str() << "/%04d.png";

    std::cout << "Saving Images without features as : " << pathResultImg2WF.str() << std::endl;
    writerImgWithoutFeatures.setFileName(pathResultImg2WF.str());
    IoverlaycolWithoutFeatures.resize(height,width);
    writerImgWithoutFeatures.open(IoverlaycolWithoutFeatures);

    tracker.getPose(cMo);
    vpPoseVector cMoVec(cMo);
    filePoseRes << cMoVec[0] << " " << cMoVec[1] << " " << cMoVec[2] << " " << cMoVec[3] << " " << cMoVec[4] << " " << cMoVec[5] << std::endl;
  }

  unsigned int iter = 1;
  // Main tracking loop
  try
  {
    while(true){
      // Render the 3D model, get the depth edges, normal and texture maps
      try{
        tracker.getPose(cMo);
        std::cout << "cMo for renderer" << std::endl;
        std::cout << cMo << std::endl;

        t0= vpTime::measureTimeMs();
        mgr->updateRTT(Inormd,Ior,&cMo);
        t1= vpTime::measureTimeMs();
        a.processEvents(QEventLoop::AllEvents, 1);
        //vpImageIo::writePNG(Inormd, "Inormd.png");
        //vpImageIo::writePNG(Ior, "Ior.png");
        tracker.Inormdprec = Inormd;
        tracker.Iorprec = Ior;
        tracker.Itexprec = Ior;
        tracker.cMoprec = cMo;

      }
      catch(...){
        vpTRACE("Error in 3D rendering");
        throw;
      }
      vpDisplay::display(Icol);

      std::cout << " disp " << im-start_image << std::endl;
      if(im-start_image>=0)
        tracker.displayRend(Icol,Inormd,Ior,vpColor::green, 1);

      vpDisplay::flush(Icol);
      vpDisplay::getImage(Icol,Ioverlaycol);

//      std::cout << "Waiting for click on Icol" << std::endl;
//      vpDisplay::getClick(Icol,true);
//      std::cout << "OK" << std::endl;


      if(!pathSave.empty())
      {
        IcolWithoutFeatures = Icol;
        vpDisplay::display(IcolWithoutFeatures);
        if(im-start_image>=0)
          tracker.displayRend(IcolWithoutFeatures,Inormd,Ior,vpColor::green, 1, false);

        vpDisplay::flush(IcolWithoutFeatures);
        vpDisplay::getImage(IcolWithoutFeatures,IoverlaycolWithoutFeatures);

        writerImg.saveFrame(Ioverlaycol);
        writerImgWithoutFeatures.saveFrame(IoverlaycolWithoutFeatures);
      }

      if(useKalmanFilter)
      {
        t0= vpTime::measureTimeMs();

        filt.cMoEst = cMo;
        filt.predictPose();
        filt.getPredPose(cMo);
        tracker.setPose(cMo);
        tracker.predictKLT = true;
        mgr->updateRTT(Inormd,Ior,&cMo);
        a.processEvents(QEventLoop::AllEvents, 1);

        t1= vpTime::measureTimeMs();
        //std::cout << "timeKalman "<< t1-t0 << std::endl;
        //timeKalman = t1-t0;
      }

      // Acquire images
      try{
        for (int sp = 0; sp < sample ; sp++)
        {
//          reader.acquire(Id);
//          readerRGB.acquire(Icol);

          g.acquire(cvI);
          vpImageConvert::convert(cvI, Icol);
          vpImageConvert::convert(cvI, Id);
        }
      }
      catch(...){
        break;
      }

      vpDisplay::display(Id);

      //tracker.setPose(cMo);
      cMo.extract(tr);
      // Pose tracking
      try{
        t0= vpTime::measureTimeMs();
        tracker.track(Id,Icol,Inormd,Ior,Ior,tr[2]);

        //tracker.displayKltPoints(Id);

        //tracker.track(Id, Igrad, Igradx, Igrady, Inormd, Ior, Ior, tr[2]);
        {
          tracker.getCovarianceMatrix(covMat);
          tracker.getCovarianceMatrixME(covMatME);
        }

        if (useKalmanFilter)
        {
          tracker.getPose(cMo);
          tracker.display(Id,cMo,mcam,vpColor::red,1);
          tracker.getCovarianceMatrix(covMat);
          tracker.getCovarianceMatrixME(covMatME);
          filt.estimatePose(cMo,covMat);
          tracker.setPose(filt.cMoEst);
          cMoFilt = filt.cMoEst;
        }

        t1= vpTime::measureTimeMs();
        //std::cout << "timeTrack "<<t1-t0 + timeKalman << std::endl;

        //if(im>1000)
        {
          meantime += (t1-t0);
          //std::cout << " mean " << (double)meantime/(double)(meant) << std::endl;
          meant++;
        }
      }
      catch(...){
        vpTRACE("Error in tracking") ;
        throw;
      }

      // Display 3D model
      tracker.getPose(cMo);

      //tracker.computeError(error);
      tracker.display(Id,cMo,mcam,vpColor::green,1);

      std::cout<<"Iter: " << iter << std::endl;
      std::cout<<" cMo out" << cMo <<std::endl;
      std::cout<<" cMo filt" << cMoFilt <<std::endl;
      iter++;

      vpDisplay::flush(Id);
      vpDisplay::getImage(Id,Ioverlay);
      //vpDisplay::getClick(Id);

      if(!pathSave.empty())
      {
        vpPoseVector cMoVec(cMo);
        filePoseRes << cMoVec[0] << " " << cMoVec[1] << " " << cMoVec[2] << " " << cMoVec[3] << " " << cMoVec[4] << " " << cMoVec[5] << std::endl;
      }

      im++;
    }
  }
  catch ( char const *e)
  {
    std::cerr << "Exception: " << e << "\n";
    return 1;
  }

  delete mgr;

  return EXIT_SUCCESS;


}

