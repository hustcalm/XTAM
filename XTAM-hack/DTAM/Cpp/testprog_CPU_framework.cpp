#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <unistd.h>

//Mine
#include "convertAhandaPovRayToStandard.h"
#include "CostVolume/utils/reproject.hpp"
#include "CostVolume/utils/reprojectCloud.hpp"
#include "CostVolume/Cost.hpp"
#include "graphics.hpp"
#include "set_affinity.h"
#include "Track/Track.hpp"
#include "utils/utils.hpp"

//debug
#include "tictoc.h"
const static bool valgrind = 0;
const static bool usePovRay = true;

//A test program to make the mapper run
using namespace cv;
using namespace std;

int App_main( int argc, char** argv );


static void myshow(const string name, const Mat& _mat){
    Mat mat = _mat.clone();
    double min;
    double max;
    cv::minMaxIdx(mat, &min, &max);
    float scale = 1 / (max-min);
    mat.convertTo(mat, CV_32FC1, scale, -min*scale);
    mat.convertTo(mat, CV_8UC3, 255.0); //use 8 bit so we can have the nice mouse over
    cout<<name<<": view scale: "<<max-min<<endl;
    namedWindow(name, 1 );
    imshow(name, mat);
}

int main( int argc, char** argv ){
    //namedWindow("backtrans",CV_WINDOW_OPENGL*0);
    set_affinity(1);
    //cvStartLoop(&App_main,argc, argv); //will crash if used with opengl!
    initGui();
    return App_main(argc, argv);
}

int App_main( int argc, char** argv )
{

    pthread_setname_np(pthread_self(),"App_main");
        
    FileStorage fs;

    Mat cameraAffinePoseBase;
    int imageNum = 0;
    char filename[500];
    Mat cameraMatrix,R,T;

    // Choose the dataset to run the mapping algorithm
    if(usePovRay) {
        convertAhandaPovRayToStandard("DTAM/Trajectory_30_seconds",
                                      imageNum,
                                      cameraMatrix,
                                      R,
                                      T);
    }
    else {
        convertPTAMSequenceToStandard("./imageSequences",
                                          imageNum+1,
                                          cameraMatrix,
                                          R,
                                          T);
    }

//     cout<<"cameraMatrix: "<<cameraMatrix<<"\n";
//     cout<< "R : "<<R<<"\n";
//     cout<< "T : "<<T<<"\n";

    if(usePovRay) {
        sprintf(filename,"DTAM/Trajectory_30_seconds/scene_%03d.png",imageNum);
    }
    else {
        sprintf(filename, "./imageSequences/image_%03d.png", imageNum+1);
    }

    Mat image;

    if(!valgrind) {
        imread(filename, -1).convertTo(image, CV_32FC3, 1.0/65535.0);   // Read the file
    }
    else {
        image.create(480, 640, CV_32FC3);
        image = 0.5;
    }
    
    pfShow("Origin Image", image, 0, Vec2d(0,1));

    hconcat(R, T, cameraAffinePoseBase); // First image with imageNum=0
   
    Cost cost(image.clone(), 128, cameraMatrix, R,T); // Cost Volume with 128 layers
    Cost cost2(image.clone(), 32, cameraMatrix, R,T);
    Track tracker(cost);
    assert(cost.rows == 480);

    vector<Mat> images,Rs,Ts;
    for(int i = 0; i <= 50; i++){
        if(usePovRay) {
            sprintf(filename, "DTAM/Trajectory_30_seconds/scene_%03d.png",i);
            convertAhandaPovRayToStandard("DTAM/Trajectory_30_seconds",
                                          i,
                                          cameraMatrix,
                                          R,
                                          T);
        }
        else {
            sprintf(filename, "./imageSequences/image_%03d.png", i+1);
            convertPTAMSequenceToStandard("./imageSequences",
                                          i+1,
                                          cameraMatrix,
                                          R,
                                          T);
        }

        Mat image;
        cout<<filename<<endl;

        if (!valgrind){
            imread(filename, -1).convertTo(image, CV_32FC3, 1.0/65535.0);
        }else{
            image.create(480, 640, CV_32FC3);
            image = 0.5;
        }

        images.push_back(image.clone());
        Rs.push_back(R.clone());
        Ts.push_back(T.clone());
    }
    
    while(1){
        for (int imageIdx = 1;imageIdx <= 50; imageIdx++){

            Mat R,T;
            T = Ts[imageIdx];
            R = Rs[imageIdx];
            image = images[imageIdx];

            Mat cameraAffinePoseAlternate, mask;
            hconcat(R,T,cameraAffinePoseAlternate); // Camera Pose with imageIdx=i

            if (cost.imageNum < 51){
                cost.updateCostL1(image, R, T); // Update cost volume
            }
            if (cost.imageNum == 50){ 
                cost.initOptimization();
                cost.optimize(); //Launches the optimizer threads
                //while(cost.running) {usleep(1000);};
            }

            
            const Mat thisPose(cost.convertPose(R, T));
            
//             reprojectCloud(image,cost.baseImage, cost._d*cost.depthStep, Mat(cost.pose), thisPose, Mat(cost.cameraMatrix));
            
            
            if(imageIdx == 1){
                tracker.pose = tracker.basePose.clone();
            }
            //Test out the Tracker
            {
                Mat tp;
                RTToLie(R,T,tp); 
                //tracker.pose=tp.clone();//Give the answer
                tracker.depth = abs(cost.depthMap());

                tracker.addFrame(image);
                
                tracker.align();      

                Mat p=tracker.pose;
                cout << "**********Image: "<< imageIdx << "**********" << endl;
                cout << "True Pose: "<< tp << endl;
                cout << "True Delta: "<< LieSub(tp, tracker.basePose) << endl;
                cout << "Recovered Pose: "<< p << endl;
                cout << "Recovered Delta: "<< LieSub(p,tracker.basePose) << endl;
                cout << "Pose Error: "<< p-tp << endl;
                cout << "**********Image: "<< imageIdx << "**********" << endl;
            }

            if(!image.data)                              // Check for invalid input
            {
                cout <<  "Could not open or find the image" << std::endl ;
                return -1;
            }
        }
        break; // Test for one depth map estimation
    }

    while(1) {
        usleep(1000);
    }

    return 0;
}

