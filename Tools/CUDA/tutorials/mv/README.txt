Matrix-Vector Multiplication
Y=A*X, where A is a M-by-N matrix, X is a N-element vector (or N-by-1 matrix), the result Y should be a M-element vector (or M-by-


No arguments: The application will create a randomized matrix A and a vector X. A CPU implementation of the matrix-vector multiplication will be used to generate a correct solution which will be compared with your programs output. If it matches (within a certain tolerance), if will print out "Test PASSED" to the screen before exiting.

One argument: The application will use the random initialization to create the input matrices, and write the device-computed output to the file specified by the argument.

Three arguments: The application will read input matrices from provided files. The first argument should be a file containing two integers. The first and second integers will be used as A.height and A.width, respectively. The second and third function arguments will be expected to be files which have exactly enough entries to fill matrices A and X respectively. No output is written to file.

Four arguments: The application will read its inputs using the files provided by the first three arguments, and write its output to the file provided in the fourth. Note that if you wish to use the output of one run of the application as an input, you must delete the first line in the output file, which displays the accuracy of the values within the file. The value is not relevant for this application.
