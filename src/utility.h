#ifndef UTILITY_H
#define UTILITY_H

#include <armadillo>
#include <matrix.h>
#include <mat.h>


using namespace arma;
using namespace std;

class utility
{
public:
    utility();
    void stdVectorMatrix2matlabMatrix(vector<vector<double> > *input, mxArray *outputMatrix);
    void writeMatlabFile(mat armaMatrix, const char *name, const char *filename);
    void armadillo2matlabMatrix(mat *armaMatrix, mxArray *outputMatrix, int num_elem);
};

#endif // UTILITY_H
