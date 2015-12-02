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
