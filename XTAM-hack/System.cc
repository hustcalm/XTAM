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
#include "DTAM/Cpp/CostVolume/Cost.h"
#include "DTAM/Cpp/CostVolume/CostVolume.hpp"
#include "DTAM/Cpp/Optimizer/Optimizer.hpp"
#include "DTAM/Cpp/DepthmapDenoiseWeightedHuber/DepthmapDenoiseWeightedHuber.hpp"
#include "DTAM/Cpp/graphics.hpp"
#include "DTAM/Cpp/set_affinity.h"
#include "DTAM/Cpp/Track/Track.hpp"

#include "DTAM/Cpp/utils/utils.hpp"
#include "DTAM/Cpp/tictoc.h"

const static bool valgrind=0;

//A test program to make the mapper run
using namespace cv;
using namespace cv::gpu;

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

void System::RunDTAM()
{
  static int testFramesNum = 0;
  vector<cv::Mat> testFrames;
  vector<SE3<> > testFramePoses;
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
      /*
      cv::Mat R, T, cameraMatrix;
      int layers=32;
      int imagesPerCV=20;
      //CostVolume cv(images[0], (FrameID)0, layers, 0.015, 0.0, Rs[0], Ts[0],cameraMatrix);
      */
      if(mpTracker->isInitialDone()) {
          SE3<> se3CamFromWorld;
          se3CamFromWorld = mpTracker->GetCurrentPose();
          testFramePoses.push_back(se3CamFromWorld);
          testFrames.push_back(mimFrame.clone());
          testFramesNum++;
          if(testFramesNum <= 600) {
              //mbDone=  true;
          
          sprintf(fileName,"./imageSequences/image_%03d.png", testFramesNum);
          cv::imwrite(fileName, mimFrame);

          sprintf(fileName,"./imageSequences/image_%03d.txt", testFramesNum);
          std::ofstream poseFile (fileName);
          poseFile<<se3CamFromWorld;
          poseFile.close();
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

    // Test XTAM using the collected sequences
    int len = testFrames.size();
    cout<<"Collect " << len << "Frames from PTAM" << endl;

    int numImg = len;

    Mat image, cameraMatrix, R, T;
    vector<Mat> images,Rs,Ts,Rs0,Ts0;
    Mat ret; //a place to return downloaded images to

    double reconstructionScale=5/5.;

    for(int i=0;i<numImg;i++){
        Mat image;
        
        // Get R and T from SE3<>
        R *= 0;
        T *= 0;
        images.push_back(testFrames[i].clone());
        Rs.push_back(R.clone());
        Ts.push_back(T.clone());
        Rs0.push_back(R.clone());
        Ts0.push_back(T.clone());
    }

    CudaMem cret(images[0].rows,images[0].cols,CV_32FC1);
    ret=cret.createMatHeader();
    //Setup camera matrix
    double sx=reconstructionScale;
    double sy=reconstructionScale;
    cameraMatrix+=(Mat)(Mat_<double>(3,3) <<    0.0,0.0,0.5,
                                                0.0,0.0,0.5,
                                                0.0,0.0,0.0);
    cameraMatrix=cameraMatrix.mul((Mat)(Mat_<double>(3,3) <<    sx,0.0,sx,
                                                                0.0,sy ,sy,
                                                                0.0,0.0,1.0));
    cameraMatrix-=(Mat)(Mat_<double>(3,3) <<    0.0,0.0,0.5,
                                                0.0,0.0,0.5,
                                                0.0,0.0,0);
    int layers=32;
    int imagesPerCV=20;
    CostVolume cv(images[0],(FrameID)0,layers,0.015,0.0,Rs[0],Ts[0],cameraMatrix);

    int imageNum=0;
    
    int inc=1;
    
    cv::gpu::Stream s;
    
    for (int imageNum=1;imageNum<numImg;imageNum++){
        cout << "Image Number: " << imageNum << endl;

        if (inc==-1 && imageNum<4){
            inc=1;
        }
        T=Ts[imageNum].clone();
        R=Rs[imageNum].clone();
        image=images[imageNum];

        if(cv.count<imagesPerCV){
            
            cv.updateCost(image, R, T);
            cudaDeviceSynchronize();
//             gpause();
//             for( int i=0;i<layers;i++){
//                 pfShow("layer",cv.downloadOldStyle(i), 0, cv::Vec2d(0, .5));
//                 usleep(1000000);
//             }
        }
        else{
            cudaDeviceSynchronize();
            //Attach optimizer
            Ptr<DepthmapDenoiseWeightedHuber> dp = createDepthmapDenoiseWeightedHuber(cv.baseImageGray,cv.cvStream);
            DepthmapDenoiseWeightedHuber& denoiser=*dp;
            Optimizer optimizer(cv);
            optimizer.initOptimization();
            GpuMat a(cv.loInd.size(),cv.loInd.type());
//             cv.loInd.copyTo(a,cv.cvStream);
            cv.cvStream.enqueueCopy(cv.loInd,a);
            GpuMat d;
            denoiser.cacheGValues();
            ret=image*0;
//             pfShow("A function", ret, 0, cv::Vec2d(0, layers));
//             pfShow("D function", ret, 0, cv::Vec2d(0, layers));
//             pfShow("A function loose", ret, 0, cv::Vec2d(0, layers));
//             pfShow("Predicted Image",ret,0,Vec2d(0,1));
//             pfShow("Actual Image",ret);
            
            cv.loInd.download(ret);
            pfShow("loInd", ret, 0, cv::Vec2d(0, layers));
//                waitKey(0);
//                gpause();
            
            

            bool doneOptimizing = false; int Acount=0; int QDcount=0;
            do{
//                 cout<<"Theta: "<< optimizer.getTheta()<<endl;
//
//                 if(Acount==0)
//                     gpause();
               a.download(ret);
               pfShow("A function", ret, 0, cv::Vec2d(0, layers));
                
                

                for (int i = 0; i < 10; i++) {
                    d=denoiser(a,optimizer.epsilon,optimizer.getTheta());
                    QDcount++;
                    
//                    denoiser._qx.download(ret);
//                    pfShow("Q function:x direction", ret, 0, cv::Vec2d(-1, 1));
//                    denoiser._qy.download(ret);
//                    pfShow("Q function:y direction", ret, 0, cv::Vec2d(-1, 1));
                   d.download(ret);
                   pfShow("D function", ret, 0, cv::Vec2d(0, layers));
                }
                doneOptimizing=optimizer.optimizeA(d,a);
                Acount++;
            }while(!doneOptimizing);
//             optimizer.lambda=.05;
//             optimizer.theta=10000;
//             optimizer.optimizeA(a,a);
            optimizer.cvStream.waitForCompletion();
            a.download(ret);
               pfShow("A function loose", ret, 0, cv::Vec2d(0, layers));
//                gpause();
//             cout<<"A iterations: "<< Acount<< "  QD iterations: "<<QDcount<<endl;
//             pfShow("Depth Solution", optimizer.depthMap(), 0, cv::Vec2d(cv.far, cv.near));
//             imwrite("outz.png",ret);
            
            Track tracker(cv);
            Mat out=optimizer.depthMap();
            double m;
            minMaxLoc(out,NULL,&m);
            tracker.depth=out*(.66*cv.near/m);
            if (imageNum+imagesPerCV+1>=numImg){
                inc=-1;
            }
            imageNum-=imagesPerCV+1-inc;
            for(int i=imageNum;i<numImg&&i<=imageNum+imagesPerCV+1;i++){
                tracker.addFrame(images[i]);
                tracker.align();
                LieToRT(tracker.pose,R,T);
                Rs[i]=R.clone();
                Ts[i]=T.clone();
                
                Mat p,tp;
                p=tracker.pose;
                tp=RTToLie(Rs0[i],Ts0[i]);
                {//debug
                    cout << "True Pose: "<< tp << endl;
                    cout << "True Delta: "<< LieSub(tp,tracker.basePose) << endl;
                    cout << "Recovered Pose: "<< p << endl;
                    cout << "Recovered Delta: "<< LieSub(p,tracker.basePose) << endl;
                    cout << "Pose Error: "<< p-tp << endl;
                }
                cout<<i<<endl;
                cout<<Rs0[i]<<Rs[i];
                reprojectCloud(images[i],images[cv.fid],tracker.depth,RTToP(Rs[cv.fid],Ts[cv.fid]),RTToP(Rs[i],Ts[i]),cameraMatrix);
            }
            cv=CostVolume(images[imageNum],(FrameID)imageNum,layers,0.015,0.0,Rs[imageNum],Ts[imageNum],cameraMatrix);
            s=optimizer.cvStream;
//             for (int imageNum=0;imageNum<numImg;imageNum=imageNum+1){
//                 reprojectCloud(images[imageNum],images[0],optimizer.depthMap(),RTToP(Rs[0],Ts[0]),RTToP(Rs[imageNum],Ts[imageNum]),cameraMatrix);
//             }
            a.download(ret);
            
        }
        s.waitForCompletion();// so we don't lock the whole system up forever
    }
    s.waitForCompletion();
    Stream::Null().waitForCompletion();
}

void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if(sCommand=="quit" || sCommand == "exit")
    static_cast<System*>(ptr)->mbDone = true;
}

