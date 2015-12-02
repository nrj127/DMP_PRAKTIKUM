#include "utility.h"

utility::utility()
{

}

void utility::stdVectorMatrix2matlabMatrix(vector < vector<double> > *input, mxArray *outputMatrix)
{
    // outputMatrix = mxCreateDoubleMatrix(input->size(), input->size()->size(), mxREAL);

    int mrows = input->size();
    int ncols = input->at(0).size();


    // copy each row to outputMatrix
    for (int i=0; i < mrows; i++)
    {
        memcpy(mxGetPr(outputMatrix) + i*ncols, &(input->at(i).at(0)), sizeof(double) * input->at(0).size());
    }

    //memcpy(mxGetPr(outputMatrix), &input[0][0], sizeof(double) * input->size() * input->at(0).size());
}

void utility::writeMatlabFile(mat armaMatrix, const char *name, const char *filename)
{
    MATFile *pmat;
    mxArray *matlabMatrix;

    matlabMatrix = mxCreateDoubleMatrix(armaMatrix.n_rows, armaMatrix.n_cols, mxREAL);
    armadillo2matlabMatrix(&armaMatrix, matlabMatrix, armaMatrix.n_elem);

    pmat=matOpen(filename, "w");
    matPutVariable(pmat, name, matlabMatrix);
    matClose(pmat);

    mxDestroyArray(matlabMatrix);
}

void utility::armadillo2matlabMatrix(mat *armaMatrix, mxArray *outputMatrix, int num_elem)
{
    int mrows = armaMatrix->n_rows;
    int ncols = armaMatrix->n_cols;
    double *src = armaMatrix->memptr();
    double *dest = mxGetPr(outputMatrix);
    memcpy(dest, src, sizeof(double)*num_elem);
}

utility::~utility()
{

}
