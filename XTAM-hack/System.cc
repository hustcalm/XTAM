// Copyright 2008 Isis Innovation Limited
#include "System.h"
#include "OpenGL.h"
#include <gvars3/instances.h>
#include <stdlib.h>
#include "ATANCamera.h"
#include "MapMaker.h"
#include "DenseMapMaker.h"
#include "Tracker.h"
#include "ARDriver.h"
#include "MapViewer.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>

#include "DTAM/Cpp/CostVolume/utils/reproject.hpp"
#include "DTAM/Cpp/CostVolume/utils/reprojectCloud.hpp"
#include "DTAM/Cpp/CostVolume/Cost.hpp"
#include "DTAM/Cpp/CostVolume/CostVolume.hpp"
#include "DTAM/Cpp/Optimizer/Optimizer.hpp"
#include "DTAM/Cpp/DepthmapDenoiseWeightedHuber/DepthmapDenoiseWeightedHuber.hpp"
#include "DTAM/Cpp/graphics.hpp"
#include "DTAM/Cpp/set_affinity.h"
#include "DTAM/Cpp/Track/Track.hpp"

#include "DTAM/Cpp/utils/utils.hpp"
#include "DTAM/Cpp/tictoc.h"

#include <pthread.h>

const static bool valgrind = 0;
static bool collectOfflineImageSequenceMode = false;
static bool collectDenseInitialFrames = true;

//A test program to make the mapper run
using namespace cv;
using namespace cv::gpu;

// Thread stuff
int init_thread_id;
pthread_t init_thread;


void* testPthread(void* ptr) {
    cout<<"Hello Pthread from XTAM..."<<endl;
}

void* dense_init_func(void* ptr) {
}

/**
 * The default System contructor.
 */
System::System()
  : mGLWindow(mVideoSource.Size(), "PTAM")
{
  // Register callback functions to GUI
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  
  // Resize the BW and RGB frames according to the video source size
  mimFrameBW.resize(mVideoSource.Size());
  mimFrameRGB.resize(mVideoSource.Size());

  // First, check if the camera is calibrated.
  // If not, we need to run the calibration widget.
  TooN::Vector<NUMTRACKERCAMPARAMETERS> vTest;
  
  // Get camera parameters through GV3
  vTest = GV3::get<TooN::Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);

  // Spawn threads for initialization and other stuff maybe
  init_thread_id = pthread_create(&init_thread, NULL, testPthread, NULL);
  if(init_thread_id) {
      cout<<"Error, pthread_create() return code:"<<init_thread_id<<endl;
  }

  // Construct a live camera
  cout << "Constucting the camera..." << endl;
  mpCamera = new ATANCamera("Camera");
  cout << "Done!" << endl;

  // I believe the below 2 lines are for test of '==' of vector
  //Vector<2> v2;
  //if(v2 == v2) ;

  // Camera is not calibrated yet, give hints and exit the program
  if(vTest == ATANCamera::mvDefaultParams)
    {
      cout << endl;
      cout << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << endl;
      cout << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << endl;
      exit(1);
    }
  
  // Let's dance! But first contruct the whole world!
  // Note once construted, they are living objects in the system
  // The MapMaker object will live in its own thread
  mpMap = new Map;
  mpMapMaker = new MapMaker(*mpMap, *mpCamera);
  //mpDenseMapMaker = new DenseMapMaker(*mpMap, *mpCamera);
  mpTracker = new Tracker(mVideoSource.Size(), *mpCamera, *mpMap, *mpMapMaker);
  mpARDriver = new ARDriver(*mpCamera, mVideoSource.Size(), mGLWindow);
  mpMapViewer = new MapViewer(*mpMap, mGLWindow);
  
  // Draw the menus out on the GUI using ParseLine
  // Note that the 'GUI' object is made live in 'gvars3/inst.cc'
  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
  GUI.ParseLine("DrawAR=0");
  GUI.ParseLine("DrawMap=0");
  GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");
  
  mbDone = false;

};

void System::Run()
{
  while(!mbDone)
    {
      
      // We use two versions of each video frame:
      // One black and white (for processing by the tracker etc)
      // and one RGB, for drawing.

      // Grab new video frame...
      mVideoSource.GetAndFillFrameBWandRGBDTAM(mimFrameBW, mimFrameRGB, mimFrame);  

      static bool bFirstFrame = true;
      if(bFirstFrame)
	  {
          // First frame arrived, initialize the ARDriver
          mpARDriver->Init();
          bFirstFrame = false;
	  }
      
      // As we invoke the blow 3 lines for every frame
      // this should be related to each camera frame for display
      mGLWindow.SetupViewport();
      mGLWindow.SetupVideoOrtho();
      mGLWindow.SetupVideoRasterPosAndZoom();
      
      // If the map is not good, then reset the ARDriver
      if(!mpMap->IsGood())
	      mpARDriver->Reset();
      
      static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN|SILENT);
      static gvar3<int> gvnDrawAR("DrawAR", 0, HIDDEN|SILENT);
      
      // Whether we will draw the map or draw the AR
      bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
      bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;
      
      // Track the frame
      mpTracker->TrackFrame(mimFrameBW, !bDrawAR && !bDrawMap);
      
      // Draw map or AR on the window
      if(bDrawMap)
	      mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
      else if(bDrawAR)
	      mpARDriver->Render(mimFrameRGB, mpTracker->GetCurrentPose());

      // mGLWindow.GetMousePoseUpdate();
      
      // Draw caption
      string sCaption;
      if(bDrawMap)
	      sCaption = mpMapViewer->GetMessageForUser();
      else
          sCaption = mpTracker->GetMessageForUser();

      mGLWindow.DrawCaption(sCaption);
      mGLWindow.DrawMenus();
      mGLWindow.swap_buffers();
      mGLWindow.HandlePendingEvents();
    }
}

vector<cv::Mat> initFrames;
vector<cv::Mat> initPoses;
int initFramesToCollect = 50;
int initFrameNum = 0;

void System::RunDTAM()
{
  static int testFramesNum = 0;
  vector<cv::Mat> testFrames;
  vector<SE3<> > testFramePoses;

  typedef struct tagDepthRange {
      float near;
      float far;
  }depthRange;

  char fileName[500];
  while(!mbDone)
    {
      
      // We use two versions of each video frame:
      // One black and white (for processing by the tracker etc)
      // and one RGB, for drawing.

      // Grab new video frame...
      mVideoSource.GetAndFillFrameBWandRGBDTAM(mimFrameBW, mimFrameRGB, mimFrame);  
      // Show original RGB Frame
      pfShow("DTAM", mimFrame);

      static bool bFirstFrame = true;
      if(bFirstFrame)
	  {
          // First frame arrived, initialize the ARDriver
          mpARDriver->Init();
          bFirstFrame = false;
	  }
      
      // As we invoke the blow 3 lines for every frame
      // this should be related to each camera frame for display
      mGLWindow.SetupViewport();
      mGLWindow.SetupVideoOrtho();
      mGLWindow.SetupVideoRasterPosAndZoom();
      
      // If the map is not good, then reset the ARDriver
      if(!mpMap->IsGood())
	      mpARDriver->Reset();
      
      static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN|SILENT);
      static gvar3<int> gvnDrawAR("DrawAR", 0, HIDDEN|SILENT);
      
      // Whether we will draw the map or draw the AR
      bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
      bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;
      
      // Track the frame
      mpTracker->TrackFrame(mimFrameBW, !bDrawAR && !bDrawMap);
      //mpTracker->TrackFrame(mimFrameBW, mimFrame, !bDrawAR && !bDrawMap);

      /*
      cv::Mat R, T, cameraMatrix;
      int layers=32;
      int imagesPerCV=20;
      CostVolume cv(images[0], (FrameID)0, layers, 0.015, 0.0, Rs[0], Ts[0], cameraMatrix);
      */
      if(mpTracker->isInitialDone() && collectOfflineImageSequenceMode == true) {
          SE3<> se3CamFromWorld;
          se3CamFromWorld = mpTracker->GetCurrentPose();
          testFramePoses.push_back(se3CamFromWorld);
          testFrames.push_back(mimFrame.clone());
          testFramesNum++;
          if(testFramesNum <= 600) {
              //mbDone=  true;
          
          sprintf(fileName, "./imageSequences/image_%03d.png", testFramesNum);
          cv::imwrite(fileName, mimFrame);

          sprintf(fileName, "./imageSequences/image_%03d.txt", testFramesNum);
          std::ofstream poseFile (fileName);
          poseFile<<se3CamFromWorld;
          poseFile.close();
          }
      }

      if(mpTracker->isInitialDone() && collectDenseInitialFrames == true) {
          static bool getSecondKF = true;
          if(getSecondKF == true) {
              //initFrames.push_back(mpTracker->mSecondKF);
              //initPoses.push_back(mpTracker->mSecondKF.se3CfromW);
              initFrameNum++;
              getSecondKF = false;
          }

          // Collect overlapping images
          SE3<> se3CamFromWorld;
          se3CamFromWorld = mpTracker->GetCurrentPose();
          //initPoses.push_back(se3CamFromWorld);
          //initFrames.push_back(mimFrame.clone());
          initFrameNum++;
          if(initFrameNum == initFramesToCollect) {
              collectDenseInitialFrames = false;

              // Start initial MapMaker thread of DTAM by notifying the thread 
              // and passing frames along with poses to the initial MapMaker
          }
      }

      // Draw map or AR on the window
      if(bDrawMap)
	      mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
      else if(bDrawAR)
	      mpARDriver->Render(mimFrameRGB, mpTracker->GetCurrentPose());

      // mGLWindow.GetMousePoseUpdate();
      
      // Draw caption
      string sCaption;
      if(bDrawMap)
	      sCaption = mpMapViewer->GetMessageForUser();
      else
          sCaption = mpTracker->GetMessageForUser();

      mGLWindow.DrawCaption(sCaption);
      mGLWindow.DrawMenus();
      mGLWindow.swap_buffers();
      mGLWindow.HandlePendingEvents();
    }
}

void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if(sCommand=="quit" || sCommand == "exit")
    static_cast<System*>(ptr)->mbDone = true;
}

