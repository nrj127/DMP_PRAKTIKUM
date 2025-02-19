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
    utility() { }

    static void stdVectorMatrix2matlabMatrix(vector< vector< double > > *input, mxArray *outputMatrix);
    static void writeMatlabFile(mat armaMatrix, const char *varname, const char *filename);
    static void writeMatlabFile(mxArray *matlabMatrix, const char *varname, const char *filename);
    static void armadillo2matlabMatrix(mat *armaMatrix, mxArray *outputMatrix, int num_elem);
    static vector<double> armadilloVector2stdVector(mat *armaMatrix);
    // static void writeMatlabFile(vector< mxArray* > matlabMatrixMulti, const char *filename);
    static void stdVector2matlabVector(vector<double> *input, mxArray *outputMatrix);
    static mat matlab2armadilloMatrix(mxArray *matlabMatrix);
    static cube matlab2armadilloMatrix3D(mxArray *matlabMatrix);
    static vec cvec2armadilloColVec(vector<double> input);
    // rotation matrix decomposition and composition
    static vec rotationMatrix2eulerAngles(mat r);
    static mat eulerAngles2rotationMatrix(vec e);

    static void array2mat_vec(mat& rot_pose0, vec& vec_pose0,float* pose0, int fri_cart_dim);
    static void mat_vec2array(mat &rotmat, vec &transvec, float* f_int_pose);

    ~utility() { }
};

#endif // UTILITY_H
