#include <stdlib.h>
////////////////////////////////////////////////////////////////////////////////
// export C interface
extern "C"
void computeGold( float*, const float*, const float*, unsigned int, unsigned int);

////////////////////////////////////////////////////////////////////////////////
//! Compute reference data set
//! Y = A * X
//! @param Y          reference data, computed but preallocated
//! @param A          matrix A as provided to device
//! @param X          Vector X as provided to device
//! @param hA         height of matrices A
//! @param wA         width of matrices A 
////////////////////////////////////////////////////////////////////////////////
void
computeGold(float* Y, const float* A, const float* X, unsigned int hA, unsigned int wA)
{
	
	// For each element in the result matrix matrix
	for (unsigned int i = 0; i < hA; ++i){
		Y[i] = 0;
        	for (unsigned int j = 0; j < wA; j++) 
			Y[i]+=A[i*wA+j]*X[j];
	}
}

