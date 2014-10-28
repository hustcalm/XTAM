// Following the papre, this is my best guess of how the DTAM system works.

int main()
{
    // System bootstrapping using standard point feature based stereo method, possibly PTAM
    KeyFrame first, second;
    double invDepthMin, invDepthMax;
    PTAM.init(first, second, invDepthMin, invDepthMax); // This will get two keyframes and know the minimum and maximum inverse depth from sparse point cloud

    // Collect frames using PTAM Tracker for bootstrapping the first depth map of DTAM
    int numberOfFramesForDTAMInit;
    vector<Frame> initFrames;
    initFrames.push_back(second);
    PTAM.collectFramesForDTAM(numberOfFramesForDTAMInit, initFrames);

    // Compute the first depth map as the init dense model
    computeInitDenseModel(initFrames, invDepthMin, invDepthMax);

    // As long as we have the dense model, we may switch to the fully dense pipeline
    switchToDensePipeline();

    // Dense Mapping and Tracking as seperated threads
    launchOptimizeThread(); // Then the thread will run in background and optimisation iterations can be interleaved with updating the cost volume average
    while(True) {
        // Get frame
        Frame frame = getFrameFromCamera();
        denseTracker.track(frame); // Track the pose of current frame and add new keyframe as needed
    }
}

class DenseTracker{
    Frame lastFrame, currentFrame;
    DenseMap* denseModel;
};

class DenseMapper{
    CostVolume cv;
};
