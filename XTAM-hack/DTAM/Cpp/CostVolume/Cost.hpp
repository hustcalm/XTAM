#ifndef COST_H
#define COST_H
#include <opencv2/core/core.hpp>
#include <vector>
#include "tictoc.h"

// The cost volume. Conceptually arranged as an image plane, corresponding
// to the keyframe, lying on top of the actual cost volume, a 3D two channel matrix storing
// the total cost of all rays that have passed through a voxel, and the number of rays that
// have hit that voxel.
//
// There is also the depth array, which stores the inverse depths of each plane of the cost.
//
// For efficiency, the accumulated cost and hit count are seperate arrays even though
// conceptually they are just different channels of the same image.
//
// The cost volume doesn't support updating by a different camera than the one that took the
// keyframe, because that would violate a bunch of assumptions for DTAM

#define COST_H_DEFAULT_NEAR 0.015


class Cost{
public:
    cv::Mat rayHits; // Number of times a ray has been hit(not implemented)
    cv::Mat_<cv::Vec3f> baseImage; // Original Captured Image
    cv::Mat lo; // Low cost maintained for acceleration
    cv::Mat hi; // High cost
    int rows;
    int cols;
    int layers;
    std::vector<float> depth;
    float near; // Inverse depth maximum
    float far;  // Inverse depth minimum
    float depthStep; // Discretisation, determine the number of points that are sampled linearly between near and far
    cv::Matx33d cameraMatrix; // Intrinsics
    cv::Matx44d basePose; // The affine transform representing the world -> camera frame transformation
    float* data; // Stores the [rows][cols][layers] array of sum of costs so far, M*N*S
    float* hit;  // Stores the number of times each cell has been hit by a ray
    int imageNum;


    Cost(); //DANGER: only use for copying to later
    
    // Autogenerate default depths
    Cost(const cv::Mat& baseImage, int layers, const cv::Mat& cameraMatrix, const cv::Mat& R, const cv::Mat& Tr); 
    Cost(const cv::Mat& baseImage, int layers, const cv::Mat& cameraMatrix, const cv::Matx44d& cameraPose); 
    
    // Use given depths
    Cost(const cv::Mat& baseImage, const std::vector<float>& depth, const cv::Mat& cameraMatrix, const cv::Mat& R, const cv::Mat& Tr);
    Cost(const cv::Mat& baseImage, const std::vector<float>& depth, const cv::Mat& cameraMatrix, const cv::Matx44d& cameraPose);

    // Core part, update cost volume and optimizize to estimate depth map
    void updateCostL1(const cv::Mat& image, const cv::Matx44d& currentCameraPose);
    void updateCostL1(const cv::Mat& image, const cv::Mat& R, const cv::Mat& Tr);
    void updateCostL2(const cv::Mat& image, const cv::Matx44d& currentCameraPose);
    void updateCostL2(const cv::Mat& image, const cv::Mat& R, const cv::Mat& Tr);
    void optimize();
    void initOptimization();
    
    const cv::Mat depthMap(); //return the best available depth map

    const cv::Matx44d convertPose(const cv::Mat& R, const cv::Mat& Tr){
        cv::Mat pose = cv::Mat::eye(4,4, CV_64F);
        R.copyTo(pose(cv::Range(0,3), cv::Range(0,3)));  // 3*3 R
        Tr.copyTo(pose(cv::Range(0,3), cv::Range(3,4))); // 3*1 T
            
        return cv::Matx44d(pose);
    }


private:
    cv::Mat_<float> dataContainer; // Stores the actual data for data*, used for auto allocation behavior
    cv::Mat_<float> hitContainer;  // Stores the actual data for hit*, used for auto allocation behavior


    //Initializer functions
    void init(){
        assert(baseImage.data); // make sure not trying to init an imageless object
        depthStep = ((depth.back()-depth[0])/layers);
        near = depth.back();
        far  = depth.front();

        data = (float*)dataContainer.data;
        hit  = (float*)hitContainer.data;

        _a.create(rows, cols, CV_32FC1); // Auxiliary variable alpha
        aptr = _a.data;
        _d.create(rows, cols, CV_32FC1); // Inverse depth values
        dptr = _d.data;

        imageNum = 0;
        QDruncount = 0;
        Aruncount = 0;
        thetaStart = 500.0;
        thetaMin = 0.01;
        running_a = running_qd = false;

        initOptimization();

        epsilon = .1;     // The Huber norm parameter
        lambda = .000001; // The weight between data term and regulirazation term
        thetaStep = .99;  // The parameter beta, control the convergence rate
    }

    std::vector<float> generateDepths(int layers){
        std::vector<float> depths;
        for(float n = 0; n < layers; ++n){
                depths.push_back(n/(layers-1)*COST_H_DEFAULT_NEAR); // [0, COST_H_DEFAULT_NEAR]
        }

        return depths; 
    }

    
    // Utility Functions
    void minv(uchar*/*(float*)*/,cv::Mat& minIndex,cv::Mat& minValue);
    void minv(float*            ,cv::Mat& minIndex,cv::Mat& minValue);
    void maxv(float*/*(float*)*/,cv::Mat& maxIndex,cv::Mat& maxValue);   
    void minmax();    

    // DepthmapDenoiseWeightedHuber functions and data
public:
    cv::Mat _qx, _qy, _d, _a, _g, _gu, _gd, _gl, _gr, _gbig;
    
private:
    uchar* aptr;
    uchar* dptr;
    cv::Mat stableDepth;
    float theta, thetaStart, thetaStep, thetaMin, epsilon, lambda, sigma_d, sigma_q;
    
    void computeSigmas(); // The update step for Q and D
    void cacheGValues();  // The weight of Huber norm, can be cached for the keyframe
    
public: 
    void optimizeQD(); // Q update, equation (11) and (12)
    
private:
    float aBasic(float* data, float l, float ds, float d, float& value); // Equation(14)
public: 
    void optimizeA(); //A update
    
private:
    // Instrumentation
    int QDruncount;
    int Aruncount;
    
public:
    // Thread management
    volatile bool running_a, running_qd;
};

#endif
