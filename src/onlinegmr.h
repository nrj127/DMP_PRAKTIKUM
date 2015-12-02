#ifndef ONLINEGMR_H
#define ONLINEGMR_H

#include <math.h>

#include <armadillo>
#include <mat.h>
#include <matrix.h>
#include <string>
#include <vector>

// #include <matlab.h> to use matlab built in functions

// the matlab library "libmat.lib" has been added to the project in CMakeLists.txt
// in order to read and write matlab files

using namespace std;
using namespace arma;

typedef struct {
    vector< vector <mat> > Priors;
    vector< vector <mat> > Mu;
    vector< mat > Priors_mixtures;
    vector< vector <cube> > Sigma2;
} GMM;

class onlineGMR
{
private:

    const char* inputFile;
    const char* outputFile;

    static const char FIRST_LEVEL_DIMENSIONS = 9;
    static const char SIGMA_DIM_M = 4;
    static const char SIGMA_DIM_N = 4;

    static const char SIGMA_DIM_O = 25;

    mat matlab2armadilloMatrix(mxArray *matlabMatrix);
    cube matlab2armadilloMatrix3D(mxArray *matlabMatrix);

    GMM gmm;
    void debugForcingTerms(vec F);
    vec calcPDF(vec X, vec Mu, mat Sigma);

public:
    onlineGMR(const char *inputFile, const char *outputFile);


    void readMatlabFile();

    void writeMatlabFile(mat armaMatrix, const char *name);

    vector<double> regression(vec vecX);
    virtual ~onlineGMR();
    void armadillo2matlabMatrix(mat *armaMatrix, mxArray *outputMatrix, int num_elem);
    void stdVector2matlabVector(vector<double> *input, mxArray *outputMatrix);
    vector<double> armadilloVector2stdVector(mat *armaMatrix);
    void stdVectorMatrix2matlabMatrix(vector<vector<double> > *input, mxArray *outputMatrix);
};

#endif // ONLINEGMR_H
