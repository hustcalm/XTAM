//#ifdef COST_CPP_SUBPARTS
#include "updateCost.part.hpp"

#include "CostVolume/utils/reproject.hpp"

//debugs
#include <iostream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include "tictoc.h"
#include "graphics.hpp"
// #define DTAM_COST_DEBUG

//in Cost.cpp
using namespace std;
static inline float fastabs(const float& foo){
    return fabs(foo);
}

/**
 * Update cost volume according to equation (2) and (3) using L1 norm
 */
void Cost::updateCostL1(const cv::Mat& image,
                                     const cv::Matx44d& currentCameraPose)
{
    imageNum++; // New Image coming to Cost Volume
    cv::Mat newLo(rows, cols, CV_32FC1, 1000.0);
    newLo = 1000.0;

    cv::Mat newHi(rows, cols, CV_32FC1, 0.0);
    newHi = 0.0;

    for(int n = 0; n < depth.size(); ++n){ // foreach depth layer
        
        // tic();
        cv::Mat_<cv::Vec3f> plane;
        cv::Mat_<uchar> mask;
        
        // Photo-Consistency, to find the correspondence, reproject using a specified depth value
        reproject(cv::Mat_<cv::Vec3f>(image), cameraMatrix, basePose, currentCameraPose, depth[n], plane, mask);

        size_t end = image.rows*image.cols*image.channels(); // End index for pixel values
        size_t lstep = layers;

#ifdef DTAM_COST_DEBUG
        float* pdata;
#else
        const float* pdata;
#endif

        pdata = (float*)(plane.data);
        const float* idata = (float*)(baseImage.data);
        float* cdata = data + n; // Cost Volume is M*N*S, make it to MN*S
        float* hdata = hit + n;
        float* ldata = (float*)(newLo.data);

        float* xdata = (float*)(newHi.data);
        char*  mdata = (char*)(mask.data);

        //hdata and cdata aligned
        //pdata and idata aligned
        //size_t moff = 0;
        //toc();
        //tic();

        for (size_t i = 0, moff = 0, coff = 0, p = 0; 
                i < end; 
                p++, moff += 3, i += 3, coff += lstep){ // foreach pixel, Cost time: .0055 - .0060 s 

            //std::cout<<mdata[moff]<<std::endl;
            if(mdata[moff]){
                float v1 = fastabs(pdata[i] - idata[i]); // Equation (3), RGB values
                float v2 = fastabs(pdata[i+1] - idata[i+1]);
                float v3 = fastabs(pdata[i+2] - idata[i+2]);
                float h = hdata[coff] + 1; // Increment hit number, namely visible from both views and thus has correspondence
                float ns = cdata[coff]*(1 - 1/h) + (v1 + v2 + v3)/h; // Equation (2), new average
                
                hdata[coff] = h;  // Update hit

                cdata[coff] = ns; // Update cost
                
               // std::cout<<ns<<std::endl;
            }

#ifdef DTAM_COST_DEBUG
            {   // debug see the cost
                pdata[i]   = cdata[coff];
                pdata[i+1] = cdata[coff];
                pdata[i+2] = cdata[coff];
            }
#endif
        }

#ifdef DTAM_COST_DEBUG
        {  // debug
           pfShow( "Cost Volume Slice", plane, 0, cv::Vec2d(0, .5));
           // gpause();
        }
#endif
        //toc();
    }
    
//     cv::Mat loInd(rows,cols,CV_32SC1);
//     cv::Mat loVal(rows,cols,CV_32FC1);
//     minv(data,loInd,loVal);
// 
//     //         loInd.convertTo(loInd,CV_32FC1);
//     imshow( "Display window", loInd*255*255/layers);
//     //std::cout<<"Depth: "<<depth[n]<<std::endl;
//     cvWaitKey(1);
}

/**
 * Update cost volume according to equation (2) and (3) , however try using L2 norm
 */
void Cost::updateCostL2(const cv::Mat& image,
                        const cv::Matx44d& currentCameraPose)
{
    if (image.type() == CV_32FC3){
        cv::Mat newLo(rows, cols, CV_32FC1, 1000.0);
        newLo = 1000.0;
        cv::Mat lInd(rows, cols, CV_32SC1);
        lInd = 255;
        
        cv::Mat newHi(rows,cols,CV_32FC1,0.0);
        newHi = 0.0;

        for(int n = 0; n < depth.size(); ++n){ // foreach depth layer
            
            cv::Mat_<cv::Vec3f> _img(image);
            cv::Mat_<cv::Vec3f> plane;
            cv::Mat_<uchar> mask;
            
            reproject(_img, cameraMatrix, basePose, currentCameraPose, depth[n], plane, mask);

            size_t end = image.rows*image.cols*image.channels();
            size_t lstep = layers;
            float* pdata = (float*)(plane.data);
            float* idata = (float*)(baseImage.data);
            float* cdata = data + n;
            float* hdata = hit + n;
            float* ldata=(float*)(newLo.data);
            
            float* xdata=(float*)(newHi.data);
            char*  mdata=(char*)(mask.data);
            
            //size_t moff=0;
            for (size_t i = 0, moff = 0, coff = 0, p = 0;
                    i < end; 
                    p++, moff += 3, i += 3, coff += lstep){ // foreach pixel
                if (n == 0){
                    ldata[p] = 255.0;
                }
                //std::cout<<mdata[moff]<<std::endl;
                if(mdata[moff]){
                    float v1 = pdata[i] - idata[i];
                    float v2 = pdata[i+1] - idata[i+1];
                    float v3 = pdata[i+2] - idata[i+2];
                    float ns = cdata[coff] + v1*v1+v2*v2+v3*v3;
                    cdata[coff] = ns;
                    
                    if(ldata[p] > (ns/hdata[coff])){
                        ldata[p] = ns/hdata[coff];
                        
                        ((int*)(lInd.data))[p] = n;
                    }
                    
                    hdata[coff] += 1.0;
                    
                    //std::cout<<p<<std::endl;
                }
                //                 {
                //                 // debug see the cost
                //                 pdata[i]=cdata[coff]/hdata[coff]*10;
                //                 pdata[i+1]=cdata[coff]/hdata[coff]*10;
                //                 pdata[i+2]=cdata[coff]/hdata[coff]*10;
                //                 }
            }
            //             {
            //             // debug
            //             absdiff(plane,baseImage,plane);
            //             cv::Mat tmp;
            //             lInd.convertTo(tmp, CV_32FC3, 1./255.);
            //             imshow( "Display window", tmp);
            //             std::cout<<"Depth: "<<depth[n]<<std::endl;
            //             cvWaitKey(1);
            //             }
        }
    }
    else if (image.type() == CV_32FC1){ // Grayscale image
        for(int n = 0; n < depth.size(); ++n){
            cv::Mat_<float> _img(image);
            cv::Mat_<float> plane;
            cv::Mat_<uchar> mask;
            
            reproject(_img, cameraMatrix, basePose, currentCameraPose, depth[n], plane, mask);

            size_t end = image.rows*image.cols;
            size_t lstep = layers;
            float* pdata = (float*)(plane.data);
            float* idata = (float*)(baseImage.data);
            float* cdata = ((float*)data) + n;
            float* hdata = ((float*)hit) + n;
            char*  mdata = (char*)(mask.data);
            
            //size_t moff=0;
            for (size_t i = 0, moff = 0, coff = 0;  
                    i<end;  
                    moff++, i++, coff += lstep) {
                //std::cout<<mdata[moff]<<std::endl;
                if(mdata[moff]){
                    float v1 = pdata[i] - idata[i];
                    cdata[coff] += v1*v1;
                    hdata[coff]++;
                    //                    {
                    //                    // debug see the cost
                    //                       pdata[i]=cdata[coff]*10;
                    //                    }
                    // std::cout<<cdata[coff]<<std::endl;
                }
            }   
        //            {
        //            // debug
        //            absdiff(plane,baseImage,plane);
        //            imshow( "Display window", plane);
        //            std::cout<<"Depth: "<<depth[n]<<std::endl;
        //            cvWaitKey(0);
        //            }
        }
    }
    else {
        std::cout<<"Error, Unsupported Type!"<<std::endl;
        assert(false);
    }
}

void Cost::updateCostL1(const cv::Mat& image, const cv::Mat& R, const cv::Mat& Tr)
{
    updateCostL1(image, convertPose(R,Tr));
}

void Cost::updateCostL2(const cv::Mat& image, const cv::Mat& R, const cv::Mat& Tr)
{
    updateCostL2(image, convertPose(R,Tr));
}
//#endif
