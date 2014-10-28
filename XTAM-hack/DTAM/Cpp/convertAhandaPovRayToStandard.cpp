// This file converts the file format used at http://www.doc.ic.ac.uk/~ahanda/HighFrameRateTracking/downloads.html
// into the standard [R|T] world -> camera format used by OpenCV
// It is based on a file they provided there, but makes the world coordinate system right handed, with z up,
// x right, and y forward.

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdio.h>

using namespace cv;
using namespace std;
Vec3f direction;
Vec3f upvector;
void convertAhandaPovRayToStandard(const char * filepath,
                                   int imageNumber,
                                   Mat& cameraMatrix,
                                   Mat& R,
                                   Mat& T)
{
    char text_file_name[600];
    sprintf(text_file_name,"%s/scene_%03d.txt",filepath,imageNumber);

    cout << "text_file_name = " << text_file_name << endl;

    ifstream cam_pars_file(text_file_name);
    if(!cam_pars_file.is_open())
    {
        cerr<<"Failed to open param file, check location of sample trajectory!"<<endl;
        exit(1);
    }

    char readlinedata[300];

    Point3d direction;
    Point3d upvector;
    Point3d posvector;


    while(1){
        cam_pars_file.getline(readlinedata,300);
//         cout<<readlinedata<<endl;
        if ( cam_pars_file.eof())
            break;


        istringstream iss;


        if ( strstr(readlinedata,"cam_dir")!= NULL){


            string cam_dir_str(readlinedata);

            cam_dir_str = cam_dir_str.substr(cam_dir_str.find("= [")+3);
            cam_dir_str = cam_dir_str.substr(0,cam_dir_str.find("]"));

            iss.str(cam_dir_str);
            iss >> direction.x ;
            iss.ignore(1,',');
            iss >> direction.z ;
            iss.ignore(1,',') ;
            iss >> direction.y;
            iss.ignore(1,',');
//             cout << "direction: "<< direction.x<< ", "<< direction.y << ", "<< direction.z << endl;

        }

        if ( strstr(readlinedata,"cam_up")!= NULL){

            string cam_up_str(readlinedata);

            cam_up_str = cam_up_str.substr(cam_up_str.find("= [")+3);
            cam_up_str = cam_up_str.substr(0,cam_up_str.find("]"));


            iss.str(cam_up_str);
            iss >> upvector.x ;
            iss.ignore(1,',');
            iss >> upvector.z ;
            iss.ignore(1,',');
            iss >> upvector.y ;
            iss.ignore(1,',');



        }

        if ( strstr(readlinedata,"cam_pos")!= NULL){
//            cout<< "cam_pos is present!"<<endl;

            string cam_pos_str(readlinedata);

            cam_pos_str = cam_pos_str.substr(cam_pos_str.find("= [")+3);
            cam_pos_str = cam_pos_str.substr(0,cam_pos_str.find("]"));

//            cout << "cam pose str = " << endl;
//            cout << cam_pos_str << endl;

            iss.str(cam_pos_str);
            iss >> posvector.x ;
            iss.ignore(1,',');
            iss >> posvector.z ;
            iss.ignore(1,',');
            iss >> posvector.y ;
            iss.ignore(1,',');
//             cout << "position: "<<posvector.x<< ", "<< posvector.y << ", "<< posvector.z << endl;

        }

    }

    R=Mat(3,3,CV_64F);
    R.row(0)=Mat(direction.cross(upvector)).t();
    R.row(1)=Mat(-upvector).t();
    R.row(2)=Mat(direction).t();

    T=-R*Mat(posvector);
//     cout<<"T: "<<T<<endl<<"pos: "<<Mat(posvector)<<endl;
   /* cameraMatrix=(Mat_<double>(3,3) << 480,0.0,320.5,
										    0.0,480.0,240.5,
										    0.0,0.0,1.0);*/
    cameraMatrix=(Mat_<double>(3,3) << 481.20,0.0,319.5,
                  0.0,480.0,239.5,
                  0.0,0.0,1.0);

}



void convertPTAMSequenceToStandard(const char * filepath,
                                   int imageNumber,
                                   cv::Mat& cameraMatrix,
                                   cv::Mat& R,
                                   cv::Mat& T) {
    char text_file_name[600];
    sprintf(text_file_name, "%s/image_%03d.txt", filepath, imageNumber);

    cout << "text_file_name = " << text_file_name << endl;

    ifstream cam_pars_file(text_file_name);
    if(!cam_pars_file.is_open())
    {
        cerr<<"Failed to open param file, check location of sample trajectory!"<<endl;
        exit(1);
    }

    // Create Mat from the PTAM SE3<> Pose
    R = Mat(3, 3, CV_64F);
    T = Mat(3, 1, CV_64F);

    char readlinedata[300];
    int currentRow = 0;
    double a, b, c, d;

    while(1){
        cam_pars_file.getline(readlinedata,300);
        cout<<readlinedata<<endl;

        if ( cam_pars_file.eof())
            break;

        istringstream iss;

        string cam_dir_str(readlinedata);

        iss.str(cam_dir_str);
        iss >> a;
        iss.ignore(1,' ');
        iss >> b;
        iss.ignore(1,' ') ;
        iss >> c;
        iss.ignore(1,' ');
        iss >> d;

        R.at<double>(currentRow, 0) = a;
        R.at<double>(currentRow, 1) = b;
        R.at<double>(currentRow, 2) = c;
        T.at<double>(currentRow, 0) = d;

        currentRow++;
    } 
    cam_pars_file.close();

    // Camear Intrinsic parameters
    double mvImageSize_0 = 640.0;
    double mvImageSize_1 = 480.0;
    double mgvvCameraParams[4] = {0.845999,1.12317,0.511039,0.458354};
    double mvFocal_0 = mvImageSize_0 * (mgvvCameraParams)[0];
    double mvFocal_1 = mvImageSize_1 * (mgvvCameraParams)[1];
    double mvCenter_0 = mvImageSize_0 * (mgvvCameraParams)[2] - 0.5;
    double mvCenter_1 = mvImageSize_1 * (mgvvCameraParams)[3] - 0.5;

    cameraMatrix=(Mat_<double>(3,3) << mvFocal_0,0.0,mvCenter_0,
                                       0.0,mvFocal_1,mvCenter_1,
                                       0.0,0.0,1.0);
}
