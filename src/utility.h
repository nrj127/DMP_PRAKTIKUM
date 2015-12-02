#ifndef UTILITY_H
#define UTILITY_H

#include <armadillo>
#include <matrix.h>


using namespace arma;
using namespace std;

class utility
{
public:
    utility();
    void stdVectorMatrix2matlabMatrix(vector<vector<double> > *input, mxArray *outputMatrix);
    void writeMatlabFile(mat armaMatrix, const char *name);
};

#endif // UTILITY_H
