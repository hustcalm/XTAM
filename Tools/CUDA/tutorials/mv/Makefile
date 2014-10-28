##############################################################
# Makefile                                                   #
#                                                            #
# Author      : Chong Ding <cding@mymail.mines.edu>          #
# Version     : 0.1                                          #
# Date        : 20100915                                     #
# Discription : generic Makefile for making cuda programs    #
##############################################################

BIN               := mv 
CXXFLAGS          := -O3 -g


CUDA_INSTALL_PATH ?= /opt/development/gpu/3.2/cuda
CUDA_SDK_PATH ?= $(HOME)/NVIDIA_GPU_Computing_SDK


NVCC ?= $(CUDA_INSTALL_PATH)/bin/nvcc
NVCC = nvcc
INCD = -I"$(CUDA_SDK_PATH)/C/common/inc" -I"$(CUDA_INSTALL_PATH)/include" -I"./" -I"/usr/lib/nvidia-cuda-toolkit/include/"
LIBS = -L"/opt/development/gpu/3.2/libcuda" -lcuda -L"$(CUDA_INSTALL_PATH)/lib64" -lcudart -lcublas -L"$(CUDA_SDK_PATH)/C/common/lib" $(CUDA_SDK_PATH)/C/lib/libcutil$(LIBSUFFIX).a -lstdc++ -lpthread
CUDA_SDK?=3
COMMONFLAGS = -DCUDA_SDK=$(CUDA_SDK)
NVCCFLAGS := --ptxas-options=-v -O3 -G -g 


# files
CPP_SOURCES       := mv_gold.cpp 
CU_SOURCES        := mv.cu
HEADERS           := $(wildcard *.h)
CPP_OBJS          := $(patsubst %.cpp, %.o, $(CPP_SOURCES))
CU_OBJS           := $(patsubst %.cu, %.cu_o, $(CU_SOURCES))

%.cu_o : %.cu
	$(NVCC) $(NVCCFLAGS) -c $(INCD) -o $@ $<

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCD) -o $@ $<

$(BIN): $(CPP_OBJS) $(CU_OBJS)
	$(CXX) -o $(BIN) $(CU_OBJS) $(CPP_OBJS) $(LDFLAGS) $(INCD) $(LIBS)

clean:
	rm -f $(BIN) *.o *.cu_o


