# DO NOT DELETE THIS LINE -- make depend depends on it.


# Edit the lines below to point to any needed include and link paths
# Or to change the compiler's optimization flags
CC = g++ -ggdb
CXX = g++
NVCC = /usr/local/cuda/bin/nvcc

MY_CUSTOM_INCLUDE_PATH = DTAM/Cpp
CUDA_INCLUDE_PATH = /usr/local/cuda/include
OPENCV_INCLUDE_PATH = /opt/OpenCV/include
OPENCV_INCLUDE_PATH1 = /opt/OpenCV/include/opencv
OPENCV_INCLUDE_PATH2 = /opt/OpenCV/include/opencv2

CUDA_LIB_PATH1 = /usr/local/cuda/lib
CUDA_LIB_PATH2 = /usr/local/cuda/lib64
CUDA_LD_PATH = /usr/local/cuda/lib64

OPENCV_LIB_PATH = /opt/OpenCV/lib

COMPILEFLAGS = -I$(MY_CUSTOM_INCLUDE_PATH) -I$(CUDA_INCLUDE_PATH) -I$(OPENCV_INCLUDE_PATH) -I$(OPENCV_INCLUDE_PATH1) -I$(OPENCV_INCLUDE_PATH2) -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3
DTAM_COMPILEFLAGS = -I$(MY_CUSTOM_INCLUDE_PATH) -I$(CUDA_INCLUDE_PATH) -I$(OPENCV_INCLUDE_PATH) -I$(OPENCV_INCLUDE_PATH1) -I$(OPENCV_INCLUDE_PATH2) -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3
DTAM_CUDA_COMPILEFLAGS = -I$(OPENCV_INCLUDE_PATH) -gencode arch=compute_30,code=sm_30 \
						 -gencode arch=compute_35,code=sm_35 \
					     --use_fast_math \
						 --restrict \
						 -lineinfo \
						 -O3

LINKFLAGS = -L/usr/local/lib -lGVars3 -lcvd -lGLU -lGL -llapack 
DTAM_LINKFLAGS = -L/usr/local/lib -L$(OPENCV_LIB_PATH) -lopencv_highgui -lopencv_core -lopencv_calib3d -lopencv_imgproc -lopencv_gpu -lpthread -lboost_thread -lboost_system
DTAM_CUDA_LINKFLAGS = -L/usr/local/lib -L$(CUDA_LIB_PATH2) -lcudart -Wl,-R $(CUDA_LD_PATH)

# Edit this line to change video source
#VIDEOSOURCE = VideoSource_Linux_V4L.o
VIDEOSOURCE = VideoSource_Linux_OpenCV.o

OBJECTS=	main.o\
		GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		System.o \
		ATANCamera.o\
		KeyFrame.o\
		MapPoint.o\
		Map.o\
		SmallBlurryImage.o\
		ShiTomasi.o \
		HomographyInit.o \
		MapMaker.o \
		Bundle.o \
		PatchFinder.o\
		Relocaliser.o\
		MiniPatch.o\
		MapViewer.o\
		ARDriver.o\
		EyeGame.o\
		Tracker.o

DTAM_SRC = DTAM/Cpp
DTAM_OBJECTS= $(DTAM_SRC)/Scheduler/ImplMutex.o\
			  $(DTAM_SRC)/graphics.o\
			  $(DTAM_SRC)/utils/ImplThreadLaunch.o\
			  $(DTAM_SRC)/CostVolume/CostVolume.o\
			  $(DTAM_SRC)/CostVolume/Cost.o\
			  $(DTAM_SRC)/CostVolume/optimizer.part.o\
			  $(DTAM_SRC)/Track/Track.o\
			  $(DTAM_SRC)/Track/align.part.o\
			  $(DTAM_SRC)/DepthmapDenoiseWeightedHuber/DepthmapDenoiseWeightedHuber.o\
			  $(DTAM_SRC)/Optimizer/Optimizer.o\
			  $(DTAM_SRC)/convertAhandaPovRayToStandard.o\
			  $(DTAM_SRC)/CostVolume/utils/reproject.o\
			  $(DTAM_SRC)/CostVolume/utils/reprojectCloud.o

X_DTAM_OBJECTS= $(DTAM_SRC)/testprog.o\
			  $(DTAM_SRC)/Scheduler/ImplMutex.o\
			  $(DTAM_SRC)/graphics.o\
			  $(DTAM_SRC)/utils/ImplThreadLaunch.o\
			  $(DTAM_SRC)/CostVolume/CostVolume.o\
			  $(DTAM_SRC)/CostVolume/Cost.o\
			  $(DTAM_SRC)/CostVolume/optimizer.part.o\
			  $(DTAM_SRC)/Track/Track.o\
			  $(DTAM_SRC)/Track/align.part.o\
			  $(DTAM_SRC)/DepthmapDenoiseWeightedHuber/DepthmapDenoiseWeightedHuber.o\
			  $(DTAM_SRC)/Optimizer/Optimizer.o\
			  $(DTAM_SRC)/convertAhandaPovRayToStandard.o\
			  $(DTAM_SRC)/CostVolume/utils/reproject.o\
			  $(DTAM_SRC)/CostVolume/utils/reprojectCloud.o

X_DTAM_CPU_OBJECTS= $(DTAM_SRC)/testprog_CPU.o\
			  $(DTAM_SRC)/Scheduler/ImplMutex.o\
			  $(DTAM_SRC)/graphics.o\
			  $(DTAM_SRC)/utils/ImplThreadLaunch.o\
			  $(DTAM_SRC)/CostVolume/Cost.o\
			  $(DTAM_SRC)/CostVolume/optimizer.part.o\
			  $(DTAM_SRC)/Track/Track.o\
			  $(DTAM_SRC)/Track/align.part.o\
			  $(DTAM_SRC)/convertAhandaPovRayToStandard.o\
			  $(DTAM_SRC)/CostVolume/utils/reproject.o\
			  $(DTAM_SRC)/CostVolume/utils/reprojectCloud.o

DTAM_CUDA_SRC = DTAM/Cpp
DTAM_CUDA_OBJECTS= $(DTAM_CUDA_SRC)/CostVolume/CostVolume_cu.o\
				   $(DTAM_CUDA_SRC)/DepthmapDenoiseWeightedHuber/DepthmapDenoiseWeightedHuber_cu.o\
				   $(DTAM_CUDA_SRC)/Optimizer/Optimizer_cu.o

CALIB_OBJECTS=	GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		CalibImage.o \
		CalibCornerPatch.o\
		ATANCamera.o \
		CameraCalibrator.o

All: X_DTAM X_DTAM_CPU PTAM CameraCalibrator

PTAM: $(OBJECTS) $(DTAM_OBJECTS) $(DTAM_CUDA_OBJECTS)
	$(CC) -o PTAM $(OBJECTS) $(DTAM_OBJECTS) $(DTAM_CUDA_OBJECTS) $(LINKFLAGS) $(DTAM_LINKFLAGS) $(DTAM_CUDA_LINKFLAGS)

X_DTAM: $(X_DTAM_OBJECTS) $(DTAM_CUDA_OBJECTS)
	$(CXX) -o X_DTAM $(DTAM_CUDA_OBJECTS) $(X_DTAM_OBJECTS) $(DTAM_LINKFLAGS) $(DTAM_CUDA_LINKFLAGS)

X_DTAM_CPU: $(X_DTAM_CPU_OBJECTS)
	$(CXX) -o X_DTAM_CPU $(X_DTAM_CPU_OBJECTS) $(DTAM_LINKFLAGS)

CameraCalibrator:$(CALIB_OBJECTS)
	$(CC) -o CameraCalibrator $(CALIB_OBJECTS) $(LINKFLAGS) $(DTAM_LINKFLAGS)


%.o: %.cc
	$(CC) $< -o $@ -c $(COMPILEFLAGS)

%.o: %.cpp
	$(CC) $< -o $@ -c $(DTAM_COMPILEFLAGS)

%_cu.o: %.cu
	$(NVCC) $< -o $@ -c $(DTAM_CUDA_COMPILEFLAGS)

clean:
	rm -f *.o
	rm -f $(DTAM_SRC)/*.o
	rm -f $(DTAM_SRC)/utils/*.o
	rm -f $(DTAM_SRC)/Map/*.o
	rm -f $(DTAM_SRC)/Scheduler/*.o
	rm -f $(DTAM_SRC)/CostVolume/*.o
	rm -f $(DTAM_SRC)/CostVolume/utils/*.o
	rm -f $(DTAM_SRC)/DepthmapDenoiseWeightedHuber/*.o
	rm -f $(DTAM_SRC)/Optimizer/*.o

depend:
	rm dependecies; touch dependencies
	makedepend -fdependencies $(INCLUDEFLAGS) $(MOREINCS) *.cc *.h


-include dependencies









