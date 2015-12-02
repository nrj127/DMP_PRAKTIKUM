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
    static void stdVectorMatrix2matlabMatrix(vector<vector<double> > *input, mxArray *outputMatrix);
    static void writeMatlabFile(mat armaMatrix, const char *varname, const char *filename);
    static void writeMatlabFile(mxArray *matlabMatrix, const char *varname, const char *filename);
    static void armadillo2matlabMatrix(mat *armaMatrix, mxArray *outputMatrix, int num_elem);
    static vector<double> armadilloVector2stdVector(mat *armaMatrix);

    ~utility();
    void writeMatlabFile(vector< mxArray* > matlabMatrixMulti, const char *filename);
};

#endif // UTILITY_H
