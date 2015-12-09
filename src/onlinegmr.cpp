#include "onlinegmr.h"
#include <stdio.h>

onlineGMR::onlineGMR(const char *inputFile, const char *outputFile) : inputFile(inputFile), outputFile(outputFile)
{
    GMM gmm;
}

void onlineGMR::readMatlabFile()
{
    MATFile *pmat;
    const char* name= NULL;
    mxArray *matlabInputMatrix;

    /* open mat file and read it's content */
    pmat = matOpen( inputFile, "r");
    if (pmat == NULL)
    {
        cerr << "Error Opening File: " << name << endl;
        exit(EXIT_FAILURE);
    }

    cout << "Reading matlab file ..." << endl;

    while ((matlabInputMatrix = matGetNextVariable(pmat,&name)) != NULL)
    {
        cout << "Found variable \"" << name << endl;

        // check for variables names
        if (strcmp(name, "Mu") == 0)
        {
            vector< vector <mat> > vDMPTemp;
            for (unsigned int i=0; i < mxGetN(matlabInputMatrix); i++) {
                vector<mat> vTemp;
                for (unsigned int j=0; j < mxGetN(mxGetCell(matlabInputMatrix, i)); j++) {
                     mxArray *matlabOutputMatrix = mxGetCell(mxGetCell(matlabInputMatrix, i),j);
                     mat aM = utility::matlab2armadilloMatrix(matlabOutputMatrix);
                     vTemp.push_back(aM);
                }
                vDMPTemp.push_back(vTemp);
            }
            gmm.Mu.insert(gmm.Mu.end(), vDMPTemp.begin(), vDMPTemp.end());
        }
        else if (strcmp(name, "Priors") == 0)
        {
            vector< vector <mat> > vDMPTemp;
            for (unsigned int i=0; i < mxGetN(matlabInputMatrix); i++) {
                vector<mat> vTemp;
                for (unsigned int j=0; j < mxGetN(mxGetCell(matlabInputMatrix, i)); j++) {
                     mxArray *matlabOutputMatrix = mxGetCell(mxGetCell(matlabInputMatrix, i),j);
                     mat aM = utility::matlab2armadilloMatrix(matlabOutputMatrix);
                     vTemp.push_back(aM);
                }
                vDMPTemp.push_back(vTemp);
            }
            gmm.Priors.insert(gmm.Priors.end(), vDMPTemp.begin(), vDMPTemp.end());
        }
        else if (strcmp(name, "Priors_Mixtures") == 0)
        {
            vector< mat > vDMPTemp;
            for (unsigned int i=0; i < mxGetN(matlabInputMatrix); i++) {
                mxArray *matlabTempMatrix = mxGetCell(matlabInputMatrix, i);
                mat aM = utility::matlab2armadilloMatrix(matlabTempMatrix);
                vDMPTemp.push_back(aM);
            }
            gmm.Priors_mixtures = vDMPTemp;
        }
        else if (strcmp(name, "Sigma2") == 0)
        {
            vector< vector <cube> > vDMPTemp;
            for (unsigned int i=0; i < mxGetN(matlabInputMatrix); i++) {
                vector<cube> vTemp;
                for (unsigned int j=0; j < mxGetN(mxGetCell(matlabInputMatrix, i)); j++) {
                     mxArray *matlabOutputMatrix = mxGetCell(mxGetCell(matlabInputMatrix, i),j);
                     cube aC = utility::matlab2armadilloMatrix3D(matlabOutputMatrix);
                     vTemp.push_back(aC);
                }
                vDMPTemp.push_back(vTemp);
            }
            gmm.Sigma2.insert(gmm.Sigma2.end(), vDMPTemp.begin(), vDMPTemp.end());
        }
        else {
            cerr << "onlinegmr::readMatlabFile(): Read variable with unknown name from *.mat file!" << endl;
            exit(EXIT_FAILURE);
        }
    }
    mxDestroyArray(matlabInputMatrix);
    matClose(pmat);

    cout << "Reading GMM was successful!" << endl;
}

/*
void onlineGMR::writeMatlabFile(mat armaMatrix, const char *name)
{
    MATFile *pmat;
    mxArray *matlabMatrix;

    matlabMatrix = mxCreateDoubleMatrix(armaMatrix.n_rows, armaMatrix.n_cols, mxREAL);
    armadillo2matlabMatrix(&armaMatrix, matlabMatrix, armaMatrix.n_elem);

    pmat=matOpen(outputFile, "w");
    matPutVariable(pmat, name, matlabMatrix);
    matClose(pmat);

    mxDestroyArray(matlabMatrix);
}
*/

vec onlineGMR::calcPDF(vec X, vec Mu, mat Sigma)
{
    vec diff = X - Mu;
    int dimensions = X.size();
    // TODO add as const?
    double denominator = sqrt(pow(2*M_PI, dimensions));

    // TODO check exponential function
    return 1 / (denominator * (arma::det(Sigma))) * arma::exp(-1/2 * diff.t() * Sigma.i() * diff);
}

vector<double> onlineGMR::regression(vec X_in /* double s, vec T */)
{
    // TODO shift all declarations into members --> faster memory allocation
    // declare dimensions
    int nDMP = gmm.Priors.size();
    int nDemos = gmm.Priors[0].size();
    int kComponents = gmm.Priors[0][0].n_cols;
    int out = gmm.Mu[0][0].n_rows - 1;  // index of output element, usually 3
    int in = out -1;    // last index of input elements, usually 0 ... 2

    vec F(nDMP);    F.zeros();
    vec Ftemp(nDMP);    Ftemp.zeros();  // DEBUG
    vec currF(kComponents);
    mat InvSigma2(in,in);
    mat sumPriors(nDMP, nDemos);    sumPriors.zeros();
    vec h(kComponents);

    for (int dmp=0; dmp < nDMP; dmp++) {
        for (int i=0; i < nDemos; i++) {
            for (int k=0; k < kComponents; k++) {
                sumPriors(dmp, i) += as_scalar(gmm.Priors[dmp][i](k) * calcPDF(X_in, gmm.Mu[dmp][i].submat(0,k,in,k), gmm.Sigma2[dmp][i].slice(k).submat(0,0,in,in)));
            }
        }
        for (int i=0; i < nDemos; i++) {
            for (int k=0; k < kComponents; k++) {
                // invert Sigma Matrix
                InvSigma2 = gmm.Sigma2[dmp][i].slice(k).submat(0, 0, in, in).i();
                // calc current F term
                currF[k] = as_scalar(gmm.Mu[dmp][i](out, k) + gmm.Sigma2[dmp][i].slice(k).submat(out, 0, out, in) * InvSigma2 * (X_in - gmm.Mu[dmp][i].submat(0,0,in,0)));
                h[k] = as_scalar((gmm.Priors[dmp][i](0, k) * calcPDF(X_in, gmm.Mu[dmp][i].submat(0,k,in,k), gmm.Sigma2[dmp][i].slice(k).submat(0, 0, in, in))) / sumPriors(dmp, i));
            }
            Ftemp[dmp] = sum(h % currF); // % element wise multiplication
            cout << "Ftemp of dmp[" << dmp << "], demo[" << i << "] : " << Ftemp[dmp] << endl;

            F[dmp] += gmm.Priors_mixtures[dmp][i] * sum(h % currF);
        }
    }
    // debugForcingTerms(F);

    utility::writeMatlabFile(F, "F", outputFile);


    return utility::armadilloVector2stdVector(&F);
}

void onlineGMR::debugForcingTerms(vec F)
{
    cout << "F: " << endl << F << endl;
}

onlineGMR::~onlineGMR()
{

}
