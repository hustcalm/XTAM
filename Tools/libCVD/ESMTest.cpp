#include <cvd/image_io.h>
#include <cvd/esm.h>
#include <cvd/videodisplay.h>               //Very cheap and cheerful X window with OpenGL capabilities
#include <cvd/gl_helpers.h>                 //OpenGL wrappers for various CVD types
#include <opencv2/opencv.hpp>

using namespace CVD;
using namespace std;
using namespace cv;

int main()
{
    try 
    {
        Image<byte> templateImage = img_load("./images/image_111.png");
        Image<byte> targetImage = img_load("./images/image_112.png");

        //VideoDisplay window(templateImage.size());           //Create an OpenGL window with the dimensions of `in'
        //glDrawPixels(templateImage);

        //CameraRotation cameraRotation;
        //StaticAppearance staticAppearance;
        
        //ESMEstimator<CameraRotation, StaticAppearance> esmEstimator =  ESMEstimator<CameraRotation, StaticAppearance>(templateImage);
        const int PARAMS = 8;
        ESMEstimator<Homography<PARAMS>, StaticAppearance> esmEstimator =  ESMEstimator<Homography<PARAMS>, StaticAppearance>();
        esmEstimator.set_image(templateImage);
        esmEstimator.optimize(targetImage);
        ESMResult esmResult =  esmEstimator.get_result();
        std::cout<<"ESM Result:"<<std::endl;

        std::cout<<"Homography:"<<std::endl;
        std::cout<<esmEstimator.transform<<std::endl;

        std::cout<<"Appearance:"<<std::endl;
        std::cout<<esmEstimator.appearance<<std::endl;

        std::cout<<"Error:"<<esmResult.error<<std::endl;
        std::cout<<"Delta:"<<esmResult.delta<<std::endl;
        std::cout<<"Pixels:"<<esmResult.pixels<<std::endl;
        std::cout<<"Iterations:"<<esmResult.iterations<<std::endl;

        // Calculate from the Pose
        // Create Mat from the PTAM SE3<> Pose
        ifstream cam_pars_file0("./images/image_111.txt");
        ifstream cam_pars_file1("./images/image_112.txt");
        cv::Mat R0 = cv::Mat(3, 3, CV_64F);    
        cv::Mat T0 = cv::Mat(3, 1, CV_64F);    
        cv::Mat R1 = cv::Mat(3, 3, CV_64F);    
        cv::Mat T1 = cv::Mat(3, 1, CV_64F);    
           
        char readlinedata0[300];   
        char readlinedata1[300];   
        int currentRow = 0;       
        double a0, b0, c0, d0;        
        double a1, b1, c1, d1;        
           
        while(1){
            cam_pars_file0.getline(readlinedata0, 300);
            cam_pars_file1.getline(readlinedata1, 300);
            //cout<<readlinedata<<endl;       
           
            if (cam_pars_file0.eof())       
                break;
           
            istringstream iss0;    
         
            string cam_dir_str0(readlinedata0);
         
            iss0.str(cam_dir_str0);
            iss0 >> a0;
            iss0.ignore(1,' ');    
            iss0 >> b0;             
            iss0.ignore(1,' ') ;   
            iss0 >> c0;             
            iss0.ignore(1,' ');    
            iss0 >> d0;
         
            R0.at<double>(currentRow, 0) = a0;
            R0.at<double>(currentRow, 1) = b0;
            R0.at<double>(currentRow, 2) = c0;
            T0.at<double>(currentRow, 0) = d0;
         
            istringstream iss1;    
         
            string cam_dir_str1(readlinedata1);
         
            iss1.str(cam_dir_str1);
            iss1 >> a1;
            iss1.ignore(1,' ');    
            iss1 >> b1;             
            iss1.ignore(1,' ') ;   
            iss1 >> c1;             
            iss1.ignore(1,' ');    
            iss1 >> d1;
         
            R1.at<double>(currentRow, 0) = a1;
            R1.at<double>(currentRow, 1) = b1;
            R1.at<double>(currentRow, 2) = c1;
            T1.at<double>(currentRow, 0) = d1;

            currentRow++;         
        } 
        cam_pars_file0.close();    
        cam_pars_file1.close();    

        // Check R0,T0 and R1,T1
        cout<<"R0 and T0:"<<endl;
        cout<<R0<<endl<<T0<<endl;

        cout<<"R1 and T1:"<<endl;
        cout<<R1<<endl<<T1<<endl;

        cv::Mat Pose0;
        hconcat(R0, T0, Pose0);

        cv::Mat Pose1;
        hconcat(R1, T1, Pose1);

        cout<<"Pose0:"<<endl;
        cout<<Pose0<<endl;
        cout<<"Pose1:"<<endl;
        cout<<Pose1<<endl;

        cv::Mat_<double> bottom = (cv::Mat_<double>(1, 4) << 0,0,0,1); 
        vconcat(Pose0, bottom, Pose0);
        vconcat(Pose1, bottom, Pose1);

        cout<<"Camera Motion:"<<endl;
        cout<<Pose0*Pose1.inv()<<endl;
    }
    catch(Exceptions::All error)
    {
        std::cerr << "Error: " << error.what << std::endl;
    }

    return 0;
}
