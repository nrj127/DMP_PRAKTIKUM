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
    GMM gmm;
    GMM2 gmm2;

    void debugForcingTerms(vec F);
    void combine_data();

public:
    onlineGMR();

    // This function need to be called first in order to obtain a gmm model for the regression function
    void readMatlabFile(const char* filename);
    vector<double> regression(vec vecX);
    mat regression_jspace(vec X_in /* double s, vec T */);

    vec calcPDF(vec X, vec Mu, mat Sigma);

    // get number of dmps
    int getnDMP() const;

    virtual ~onlineGMR();
};

#endif // ONLINEGMR_H
