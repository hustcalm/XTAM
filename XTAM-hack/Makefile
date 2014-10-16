# DO NOT DELETE THIS LINE -- make depend depends on it.


# Edit the lines below to point to any needed include and link paths
# Or to change the compiler's optimization flags
CC = g++ -ggdb
CXX = g++
NVCC = nvcc

MY_CUSTOM_INCLUDE_PATH = DTAM/Cpp

COMPILEFLAGS = -I$(MY_CUSTOM_INCLUDE_PATH) -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3
DTAM_COMPILEFLAGS = -I$(MY_CUSTOM_INCLUDE_PATH) -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3
DTAM_CUDA_COMPILEFLAGS = -gencode arch=compute_30,code=sm_30 \
						 -gencode arch=compute_35,code=sm_35 \
					     --use_fast_math \
						 --restrict \
						 -lineinfo \
						 -O3

LINKFLAGS = -L /usr/local/lib -lGVars3 -lcvd -lGLU -lGL -llapack 
DTAM_LINKFLAGS = -L /usr/local/lib -lopencv_highgui -lopencv_core -lopencv_calib3d -lopencv_imgproc -lopencv_gpu -lpthread -lboost_thread -lboost_system
DTAM_CUDA_LINKFLAGS = -L /usr/local/lib -lcudart 

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

All: X_DTAM PTAM CameraCalibrator

PTAM: $(OBJECTS) $(DTAM_OBJECTS) $(DTAM_CUDA_OBJECTS)
	$(CC) -o PTAM $(OBJECTS) $(DTAM_OBJECTS) $(DTAM_CUDA_OBJECTS) $(LINKFLAGS) $(DTAM_LINKFLAGS) $(DTAM_CUDA_LINKFLAGS)

X_DTAM: $(X_DTAM_OBJECTS) $(DTAM_CUDA_OBJECTS)
	$(CXX) -o X_DTAM $(DTAM_CUDA_OBJECTS) $(X_DTAM_OBJECTS) $(DTAM_LINKFLAGS) $(DTAM_CUDA_LINKFLAGS)

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









