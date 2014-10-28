#ifndef _MATRIXMUL_H_
#define _MATRIXMUL_H_

// Thread block size
#define BLOCK_SIZE 256 
#define ITERATIONS 3000
#define TILE_WIDTH 16
// Matrix Structure declaration
typedef struct {
    unsigned int width;
    unsigned int height;
    unsigned int pitch;
    float* elements;
} Matrix;


#endif // _MATRIXMUL_H_

