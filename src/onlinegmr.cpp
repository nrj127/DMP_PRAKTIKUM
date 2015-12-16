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
    //combine_data();
}

vec onlineGMR::calcPDF(vec X, vec Mu, mat Sigma)
{
    vec diff = X - Mu;
    int dimensions = X.size();
    // TODO add as const?
    double denominator = pow(2*M_PI, dimensions);

    // TODO check exponential function
    //cout << "pdf zwischenergebnis:" << arma::exp(-1/2.0 * diff.t() * Sigma.i() * diff)  << endl;
    //cout << "1/2.0 * diff.t() * Sigma.i() * diff" << -1/2.0 * diff.t() * Sigma.i() * diff << endl;
    return 1 / sqrt((denominator * (arma::det(Sigma)))) * arma::exp(-1/2.0 * diff.t() * Sigma.i() * diff);
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
    // vec Ftemp(nDMP);    Ftemp.zeros();  // DEBUG
    vec currF(kComponents);
    mat InvSigma2(in,in);
    vec sumPriors(nDMP);    sumPriors.zeros();
    vec h(kComponents);
    cube h_debug(nDMP,nDemos,kComponents);

    for (int dmp=0; dmp < nDMP; dmp++) {
        for (int i=0; i < nDemos; i++) {
            for (int k=0; k < kComponents; k++) {
                sumPriors(dmp) += as_scalar(gmm.Priors_mixtures[dmp][i] * gmm.Priors[dmp][i](k) * calcPDF(X_in, gmm.Mu[dmp][i].submat(0,k,in,k), gmm.Sigma2[dmp][i].slice(k).submat(0,0,in,in)));                
            }
        }

        for (int i=0; i < nDemos; i++) {
            for (int k=0; k < kComponents; k++) {
                // invert Sigma Matrix
                InvSigma2 = gmm.Sigma2[dmp][i].slice(k).submat(0, 0, in, in).i();
                // calc current F term
                currF[k] = as_scalar(gmm.Mu[dmp][i](out, k) + gmm.Sigma2[dmp][i].slice(k).submat(out, 0, out, in) * InvSigma2 * (X_in - gmm.Mu[dmp][i].submat(0,k,in,k)));
                h[k] = as_scalar((gmm.Priors_mixtures[dmp][i] * gmm.Priors[dmp][i](0, k) * calcPDF(X_in, gmm.Mu[dmp][i].submat(0,k,in,k), gmm.Sigma2[dmp][i].slice(k).submat(0, 0, in, in))) / sumPriors(dmp));
            }
            F[dmp] += sum(h % currF);
        }        

    }
//    cout << "xin" << X_in << endl;
//    cout << "F "<< F << endl;

    // debugForcingTerms(F);

    utility::writeMatlabFile(F, "F", outputFile);

    return utility::armadilloVector2stdVector(&F);
}

void onlineGMR::debugForcingTerms(vec F)
{
    cout << F << endl;
}

void onlineGMR::combine_data()
{
    int nDMP = gmm.Priors.size();
    int nDemos = gmm.Priors[0].size();
    int kComponents = gmm.Priors[0][0].n_cols;


    for (int dmp=0; dmp < nDMP; dmp++) {
        //set the sizes
        vec Pr_comb;
        mat Mu;
        cube Sigma2;
        Pr_comb.zeros(kComponents*nDemos);
        Mu.zeros(4,kComponents*nDemos);
        Sigma2.zeros(4,4,kComponents*nDemos);
        gmm2.Pr_comb.push_back(Pr_comb);
        gmm2.Mu.push_back(Mu);
        gmm2.Sigma2.push_back(Sigma2);

        for (int i=0; i < nDemos; i++) {
            for (int k=0; k < kComponents; k++) {
                gmm2.Pr_comb[dmp](k+i*kComponents)=gmm.Priors[dmp][i](0,k)*gmm.Priors_mixtures[dmp][i];
                gmm2.Mu[dmp].col(k+i*kComponents)=gmm.Mu[dmp][i].col(k);
                gmm2.Sigma2[dmp].slice(k+i*kComponents)=gmm.Sigma2[dmp][i].slice(k);
            }
        }
    }

//    cout << "gmm2.Pr_comb[dmp]   "<< gmm2.Pr_comb[0] << endl;
//    cout << "gmm2.Mu[0].cols(1,10)   "<< gmm2.Mu[0].cols(0,9);
//    cout << "gmm.Mu[0][0].cols(1,10)   "<< gmm.Mu[0][0].cols(0,9);
//    cout << "gmm2.Sigma2[dmp0].slice(0)   " << gmm2.Sigma2[0].slice(0);

}

onlineGMR::~onlineGMR()
{


}
