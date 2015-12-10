#ifndef ONLINEGMR_H
#define ONLINEGMR_H

#include <math.h>

#include <armadillo>
#include <mat.h>
#include <matrix.h>
#include <string>
#include <vector>
#include "utility.h"

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

typedef struct {            //different structure identical to matlab-script
    vector <mat> Mu;        //dimensions: 3 dmps, 270 components
    vector <vec> Pr_comb;   //dimensions: 3 dmps, 270 components
    vector <cube> Sigma2;   //dimensions: 3 dmps, 270 components, 4x4 matrix
} GMM2;


class onlineGMR
{
private:
    const char* inputFile;
    const char* outputFile;
    GMM gmm;
    GMM2 gmm2;

    void debugForcingTerms(vec F);
    void combine_data();

public:
    onlineGMR(const char *inputFile, const char *outputFile);
    void readMatlabFile();
    vector<double> regression(vec vecX);
    vec calcPDF(vec X, vec Mu, mat Sigma);

    virtual ~onlineGMR();
};

#endif // ONLINEGMR_H
