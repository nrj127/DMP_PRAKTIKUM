#include "utility.h"

void utility::stdVectorMatrix2matlabMatrix(vector < vector<double> > *input, mxArray *outputMatrix)
{
    int mrows = input->size();
    int ncols = input->at(0).size();

    // copy each row to outputMatrix
    for (int i=0; i < ncols; i++)
    {
        for(int j=0; j< mrows; j++)
            memcpy(mxGetPr(outputMatrix) + i*mrows + j, &(input->at(j).at(i)), sizeof(double));
    }
}

//creates an armadillo matrix from a matlab matrix
mat utility::matlab2armadilloMatrix(mxArray *matlabMatrix)
{
    mwSize mrows = mxGetM(matlabMatrix);
    mwSize ncols = mxGetN(matlabMatrix);

    double *values = mxGetPr(matlabMatrix);

    return mat(values, mrows, ncols);
}

//creates a 3D armadillo matrix from a 3D matlab matrix
cube utility::matlab2armadilloMatrix3D(mxArray *matlabMatrix)
{
    int mrows = mxGetDimensions(matlabMatrix)[0];
    int ncols = mxGetDimensions(matlabMatrix)[1];
    int hslice = mxGetDimensions(matlabMatrix)[2];

    double *values = mxGetPr(matlabMatrix);

    return cube(values, mrows, ncols, hslice);
}

vec utility::cvec2armadilloColVec(vector<double> input)
{
    //colvec a(input.size());
    cout << "got input of size" << input.size() << endl;

    colvec a;
    //typedef std::vector<double> stdvec;
    //stdvec x(3);
    //x[0] = 0.0; x[1] = 1.0;  x[2] = 2.0;

    a = conv_to<colvec>::from(input);
    return a;
}

void utility::writeMatlabFile(mat armaMatrix, const char *varname, const char *filename)
{
    MATFile *pmat;
    mxArray *matlabMatrix;

    matlabMatrix = mxCreateDoubleMatrix(armaMatrix.n_rows, armaMatrix.n_cols, mxREAL);
    armadillo2matlabMatrix(&armaMatrix, matlabMatrix, armaMatrix.n_elem);

    pmat=matOpen(filename, "w");
    matPutVariable(pmat, varname, matlabMatrix);
    matClose(pmat);

    mxDestroyArray(matlabMatrix);
}

void utility::writeMatlabFile(mxArray *matlabMatrix, const char *varname, const char *filename)
{
    MATFile *pmat;

    pmat=matOpen(filename, "w");
    if (pmat == NULL)
        exit(EXIT_FAILURE);

    matPutVariable(pmat, varname, matlabMatrix);
    matClose(pmat);
}

void utility::armadillo2matlabMatrix(mat *armaMatrix, mxArray *outputMatrix, int num_elem)
{
    //int mrows = armaMatrix->n_rows;
    //int ncols = armaMatrix->n_cols;
    double *src = armaMatrix->memptr();
    double *dest = mxGetPr(outputMatrix);
    memcpy(dest, src, sizeof(double)*num_elem);
}

vector<double> utility::armadilloVector2stdVector(mat *armaMatrix)
{
    vector<double> TempVec(3);
    for (unsigned int i=0; i< armaMatrix->size(); i++) {
        TempVec[i] = armaMatrix->at(i);
    }
    return TempVec;
}

void utility::stdVector2matlabVector(vector<double> *input, mxArray *outputMatrix)
{
    outputMatrix = mxCreateDoubleMatrix(input->size(), 1, mxREAL);
    mxSetPr(outputMatrix, &(input->at(0)));
}


