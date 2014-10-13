// Copyright 2008 Isis Innovation Limited
#include "System.h"
#include "OpenGL.h"
#include <gvars3/instances.h>
#include <stdlib.h>
#include "ATANCamera.h"
#include "MapMaker.h"
//#include "DenseMapMaker.h"
#include "Tracker.h"
#include "ARDriver.h"
#include "MapViewer.h"

using namespace CVD;
using namespace std;
using namespace GVars3;


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
  Vector<NUMTRACKERCAMPARAMETERS> vTest;
  
  // Get camera parameters through GV3
  vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);

  // Construct a live camera
  cout << "Constucting the camera..." << endl;
  mpCamera = new ATANCamera("Camera");
  cout << "Done!" << endl;

  // I believe the below 2 lines are for test of '==' of vector
  Vector<2> v2;
  if(v2 == v2) ;

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
      mVideoSource.GetAndFillFrameBWandRGB(mimFrameBW, mimFrameRGB);  
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

void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if(sCommand=="quit" || sCommand == "exit")
    static_cast<System*>(ptr)->mbDone = true;
}

