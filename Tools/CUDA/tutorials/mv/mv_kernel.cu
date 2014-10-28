#ifndef _MV_KERNEL_H_
#define _MV_KERNEL_H_

#include <stdio.h>
#include "mv.h"

// Matrix multiplication kernel thread specification
// 1. Global Memory
__global__ void MVKernel_gm(Matrix A, Matrix X, Matrix Y)
{
	  int bx = blockIdx.x; 
          //int by = blockIdx.y;
	  int tx = threadIdx.x; 
          //int ty = threadIdx.y;
  // Calculate the row index of the Pd element and M
  int Row = bx * BLOCK_SIZE + tx;
  // Calculate the column idenx of Pd and N
  //int Col = bx * BLOCK_SIZE + tx;
  
  float Pvalue = 0;

   
  for (unsigned int k = 0; k < A.width; k++) 
    {
      if(Row < A.height)         
      Pvalue += A.elements[Row*A.width+k] * X.elements[k];
      //else
      //Pvalue += 0;

    }

  __syncthreads();
  
  if(Row < A.height)  		
    Y.elements[Row] = Pvalue;
  __syncthreads();
}


// 2. Shared Memory
__global__ void MVKernel_shm(Matrix A, Matrix X, Matrix Y)
{
   __shared__ float Xds[BLOCK_SIZE];
   
	int bx = blockIdx.x; 
        //int by = blockIdx.y;
	int tx = threadIdx.x; 
        //int ty = threadIdx.y;
  // Calculate the row index
  //int Row = by * BLOCK_SIZE + ty;
  // Calculate the column index
  int Row = bx * BLOCK_SIZE + tx;
  
  float Pvalue = 0;

  for (unsigned int m = 0; m < (A.width-1)/BLOCK_SIZE+1; ++m)
    {
      if(m*BLOCK_SIZE + tx < A.width)
      	Xds[tx] = X.elements[m*BLOCK_SIZE + tx]; 
      else
      	Xds[tx] = 0;   
      __syncthreads();	
      
      for (unsigned int k = 0; k < BLOCK_SIZE; k++) 
             if(Row<A.height && m*BLOCK_SIZE +k<A.width)
    		Pvalue += A.elements[m*BLOCK_SIZE+Row*A.width+k] * Xds[k];    		
    	__syncthreads();
    }		
    
  if(Row < A.height)  
    Y.elements[Row] = Pvalue; 
  __syncthreads();
}

#endif // #ifndef _MV_KERNEL_H_
