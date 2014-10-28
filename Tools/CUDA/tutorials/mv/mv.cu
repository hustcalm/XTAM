/* Matrix-Vector Multiplication: Y = A*X, A is the matrix, X is the Vector.
 * Host code.
 */

// includes, system
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// includes, project
//#include <cutil.h>
#include "cublas.h"

// includes, kernels
#include <mv_kernel.cu>


// declarations, forward
extern "C"

void computeGold(float*, const float*, const float*, unsigned int, unsigned int);
Matrix AllocateDeviceMatrix(const Matrix M);
Matrix AllocateMatrix(int height, int width, int init);
void CopyToDeviceMatrix(Matrix Mdevice, const Matrix Mhost);
void CopyFromDeviceMatrix(Matrix Mhost, const Matrix Mdevice);
int ReadFile(Matrix* M, char* file_name);
void WriteFile(Matrix M, char* file_name);
void FreeDeviceMatrix(Matrix* M);
void FreeMatrix(Matrix* M);
int MVOnDevice(const Matrix A, const Matrix X, Matrix Y);


char choose;
clock_t lapse;
clock_t lapse1;
/*
*******************************************************************************
* Main Program
*******************************************************************************
*/
int main(int argc, char** argv) {

  	Matrix  A;
  	Matrix  X;
  	Matrix  Y;
  	
  	srand(time(NULL));
    printf("----Please choose one method to run the matrix vector multiplication\n");
    printf("----Enter 1 or 2 or 3, then press ENTER button.\n");
    printf("----(1). Global memory\n");
    printf("----(2). shared memory\n");
    printf("----(3). CUBLAS\n");
    printf("----You choose number here: ");
    //choose = getchar();
    for (int iii = 1; iii<=3; iii++)
{   
    if (iii == 1)
      choose = '1';
    else if (iii == 2)
      choose = '2';
    else if (iii == 3)
      choose = '3';

    if (choose == '1' || choose == '2' || choose == '3') 
    	{
        printf("----Iteration number is %d:\n",ITERATIONS);	
      	if(argc != 5 && argc != 4) 
        	{
        		// Allocate and initialize the matrices
                	A  = AllocateMatrix(rand()%1024, rand()%1024, 1);
                        //A  = AllocateMatrix(4096, 4096, 1);
        		X  = AllocateMatrix(A.width, 1, 1);
        		Y  = AllocateMatrix(A.height, 1, 0);
        	}
      	else
        	{
        		// Allocate and read in matrices from disk
        		int* params = NULL; 
        		unsigned int data_read = 0;
        		cutReadFilei(argv[1], &params, &data_read, true);
        		if(data_read != 2){
        			printf("Error reading parameter file\n");
        			cutFree(params);
        			return 1;
        		}
        
        		A  = AllocateMatrix(params[0],params[1], 0);
        		X  = AllocateMatrix(A.width, 1, 0);		
        		Y  = AllocateMatrix(A.height, 1, 0);
        		cutFree(params);
        		(void)ReadFile(&A, argv[2]);
        		(void)ReadFile(&X, argv[3]);
        	}
    
    	// Matrix-Vector Multiplication on the device
    
        printf("Size of MATRIX A is %d by %d.\n",A.height,A.width);
          
        // run the computation on GPU        
        MVOnDevice(A, X, Y);
    
    
        // compute the matrix-vector multiplication on the CPU for comparison
        Matrix reference = AllocateMatrix(Y.height, Y.width, 0);
        // Measure the execution time of CPU implementation	
        lapse = clock();
        for (int i=0;i<ITERATIONS;i++)
        	computeGold(reference.elements, A.elements, X.elements, A.height, A.width);
        lapse = clock()-lapse;
    
        printf("Execution time of CPU implementation is %.6f\n", float(lapse)*1000/(ITERATIONS*CLOCKS_PER_SEC));
      
        // check if the result is equivalent to the expected soluion
        CUTBoolean res = cutComparefe(reference.elements, Y.elements, Y.width * Y.height, 0.001f);
        printf("Test %s\n", (1 == res) ? "PASSED" : "FAILED");
        //if (res != 1)
        //printf("----Because it is the paper alg. Please run agian. 90 percent time, it works well.\n");
        //printf("Please...................");
        
        if(argc == 5)
          {
    	       WriteFile(Y, argv[4]);
    	    }
    	  else if(argc == 2)
    	    {
    	       WriteFile(Y, argv[1]);
        	}   
    
    	// Free matrices
        FreeMatrix(&A);
        FreeMatrix(&X);
        FreeMatrix(&Y);
        //return 0;
      }
    else printf("Input wrong.\n");
}
}

/*
*******************************************************************************
* Run Y = A*X
*******************************************************************************
*/
int MVOnDevice(const Matrix A, const Matrix X, Matrix Y)
{
    // Load A and X  to the device
    Matrix Ad = AllocateDeviceMatrix(A);
    CopyToDeviceMatrix(Ad, A);
    Matrix Xd = AllocateDeviceMatrix(X);
    CopyToDeviceMatrix(Xd, X);
    
    // Allocate Y on the device
    Matrix Yd = AllocateDeviceMatrix(Y);
    CopyToDeviceMatrix(Yd, Y);
    // Setup the execution configuration
    //Version 1.0 Global Memory
    dim3 dimGrid((A.height-1)/BLOCK_SIZE+1);
    dim3 dimBlock(BLOCK_SIZE);    
    int m = A.height;
    //int n = A.width;
    int blkNum = (m >> 4) + ((m & 15) ? 1 : 0);
    //int height = blkNum << 4;
    //int width = (n & 255) ? (256*((n >> 8) + 1) ) : n;
    dim3 threads(16, 16);
    dim3 grid(blkNum, 1);
    //Mesaure the time of GPU implementation
    lapse1=clock();	
        for (int i=0;i<ITERATIONS;i++)
          {
              if(choose == '1')	
                {
                    if (i==0) printf("--------You choose global memory method------------\n");
                    MVKernel_gm<<<dimGrid,dimBlock>>>(Ad,Xd,Yd);
                }
              else if (choose == '2')
                {
                    if (i==0) printf("--------You choose shared memory method------------\n");
                    MVKernel_shm<<<dimGrid,dimBlock>>>(Ad,Xd,Yd);
                }
              else if (choose == '3')
                {
                    if (i==0) printf("--------You choose to use CUBLAS method------------\n");
                     cublasSgemv ('T', Ad.width, Ad.height, 1,
                                   Ad.elements, Ad.width, Xd.elements,
                                   1, 0, Yd.elements, 1);
                    cublasStatus status = cublasGetError();
                    if (status != CUBLAS_STATUS_SUCCESS) {
                        fprintf (stderr, "!!!! kernel execution error.\n");
                        return EXIT_FAILURE;
                    }    
                }
              else 
                {
                printf("You choose nothing.\n"); break;
                }

          }

    lapse1 = clock()-lapse1;
    printf("\n");
    printf("Execution time of GPU implementation is %.6f\n", float(lapse1)*1000/(ITERATIONS*CLOCKS_PER_SEC));

    // Read Y from the device
    CopyFromDeviceMatrix(Y, Yd);
      
   
    // Free device matrices
    FreeDeviceMatrix(&Ad);
    FreeDeviceMatrix(&Xd);
    FreeDeviceMatrix(&Yd);

}




// Allocate a device matrix of same size as M.
Matrix AllocateDeviceMatrix(const Matrix M)
{
    Matrix Mdevice = M;
    int size = M.width * M.height * sizeof(float);
    cudaMalloc((void**)&Mdevice.elements, size);
    return Mdevice;
}

// Allocate a device matrix of dimensions height*width
//	If init == 0, initialize to all zeroes.  
//	If init == 1, perform random initialization.
//  If init == 2, initialize matrix parameters, but do not allocate memory 
Matrix AllocateMatrix(int height, int width, int init)
{
    Matrix M;
    M.width = M.pitch = width;
    M.height = height;
    int size = M.width * M.height;
    M.elements = NULL;
    
    // don't allocate memory on option 2
    if(init == 2)
		return M;
		
	M.elements = (float*) malloc(size*sizeof(float));

	for(unsigned int i = 0; i < M.height * M.width; i++)
	{
		M.elements[i] = (init == 0) ? (0.0f) : (rand() / (float)RAND_MAX);
		if(rand() % 2)
			M.elements[i] = - M.elements[i];
	}
    return M;
}	

// Copy a host matrix to a device matrix.
void CopyToDeviceMatrix(Matrix Mdevice, const Matrix Mhost)
{
    int size = Mhost.width * Mhost.height * sizeof(float);
    Mdevice.height = Mhost.height;
    Mdevice.width = Mhost.width;
    Mdevice.pitch = Mhost.pitch;
    cudaMemcpy(Mdevice.elements, Mhost.elements, size, 
					cudaMemcpyHostToDevice);
}

// Copy a device matrix to a host matrix.
void CopyFromDeviceMatrix(Matrix Mhost, const Matrix Mdevice)
{
    int size = Mdevice.width * Mdevice.height * sizeof(float);
    cudaMemcpy(Mhost.elements, Mdevice.elements, size, 
					cudaMemcpyDeviceToHost);
}

// Free a device matrix.
void FreeDeviceMatrix(Matrix* M)
{
    cudaFree(M->elements);
    M->elements = NULL;
}

// Free a host Matrix
void FreeMatrix(Matrix* M)
{
    free(M->elements);
    M->elements = NULL;
}

// Read a floating point matrix in from file
int ReadFile(Matrix* M, char* file_name)
{
	unsigned int data_read = M->height * M->width;
	cutReadFilef(file_name, &(M->elements), &data_read, true);
	return data_read;
}

// Write a floating point matrix to file
void WriteFile(Matrix M, char* file_name)
{
    cutWriteFilef(file_name, M.elements, M.width*M.height,
                       0.0001f);
}

