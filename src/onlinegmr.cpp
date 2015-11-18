#include "onlinegmr.h"

// TODO necessary?
#include "mvn.h"
#include <stdio.h>



onlineGMR::onlineGMR(const char *inputFile, const char *outputFile) : inputFile(inputFile), outputFile(outputFile)
{
    GMM gmm;
}

//creates an armadillo matrix from a matlab matrix
mat onlineGMR::matlab2armadilloMatrix(mxArray *matlabMatrix)
{
    mwSize mrows = mxGetM(matlabMatrix);
    mwSize ncols = mxGetN(matlabMatrix);

    double *values = mxGetPr(matlabMatrix);

    return mat(values, mrows, ncols);
}

//creates a 3D armadillo matrix from a 3D matlab matrix
cube onlineGMR::matlab2armadilloMatrix3D(mxArray *matlabMatrix)
{
    int mrows = mxGetDimensions(matlabMatrix)[0];
    int ncols = mxGetDimensions(matlabMatrix)[1];
    int hslice = mxGetDimensions(matlabMatrix)[2];

    double *values = mxGetPr(matlabMatrix);

    return cube(values, mrows, ncols, hslice);
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

    while ((matlabInputMatrix = matGetNextVariable(pmat,&name)) != NULL)
    {
        cout << "Found variable \"" << name << "\" with " << (int)mxGetNumberOfDimensions(matlabInputMatrix) << " dimensions" << endl;

        // check for variables names
        if (strcmp(name, "Mu") == 0)
        {
            vector< vector <mat> > vDMPTemp;
            for (int i=0; i < mxGetN(matlabInputMatrix); i++) {
                vector<mat> vTemp;
                for (int j=0; j < mxGetN(mxGetCell(matlabInputMatrix, i)); j++) {
                     mxArray *matlabOutputMatrix = mxGetCell(mxGetCell(matlabInputMatrix, i),j);
                     mat aM = matlab2armadilloMatrix(matlabOutputMatrix);
                     vTemp.insert(vTemp.end(), aM);
                }
                vDMPTemp.insert(vDMPTemp.end(), vTemp);
                /* test output
                for(vector<mat>::iterator it = vTemp.begin(); it != vTemp.end(); ++it) {
                    //(*it).print(cout);
                }
                */
            }
            gmm.Mu.insert(gmm.Mu.begin(), vDMPTemp.begin(), vDMPTemp.end());
        }
        else if (strcmp(name, "Priors") == 0)
        {
            vector< vector <mat> > vDMPTemp;
            for (int i=0; i < mxGetN(matlabInputMatrix); i++) {
                vector<mat> vTemp;
                for (int j=0; j < mxGetN(mxGetCell(matlabInputMatrix, i)); j++) {
                     mxArray *matlabOutputMatrix = mxGetCell(mxGetCell(matlabInputMatrix, i),j);
                     mat aM = matlab2armadilloMatrix(matlabOutputMatrix);
                     vTemp.insert(vTemp.end(), aM);
                }
                vDMPTemp.insert(vDMPTemp.end(), vTemp);
                /* test output
                for(vector<mat>::iterator it = vTemp.begin(); it != vTemp.end(); ++it) {
                    //(*it).print(cout);
                }
                */
            }
            gmm.Priors.insert(gmm.Priors.begin(), vDMPTemp.begin(), vDMPTemp.end());
        }
        else if (strcmp(name, "Priors_Mixtures") == 0)
        {
            vector< vector <mat> > vDMPTemp;
            for (int i=0; i < mxGetN(matlabInputMatrix); i++) {
                vector<mat> vTemp;
                mxArray *matlabTempMatrix = mxGetCell(matlabInputMatrix, i);
                mat aM = matlab2armadilloMatrix(matlabTempMatrix);
                vTemp.insert(vTemp.end(), aM);
                vDMPTemp.insert(vDMPTemp.end(), vTemp);
                /* test output
                for(vector<mat>::iterator it = vTemp.begin(); it != vTemp.end(); ++it) {
                    (*it).print(cout);
                }
                */
            }
            gmm.Priors_mixtures.insert(gmm.Priors_mixtures.begin(), vDMPTemp.begin(), vDMPTemp.end());
        }
        else if (strcmp(name, "Sigma2") == 0)
        {
            vector< vector <cube> > vDMPTemp;
            for (int i=0; i < mxGetN(matlabInputMatrix); i++) {
                vector<cube> vTemp;
                for (int j=0; j < mxGetN(mxGetCell(matlabInputMatrix, i)); j++) {
                     mxArray *matlabOutputMatrix = mxGetCell(mxGetCell(matlabInputMatrix, i),j);
                     cube aC = matlab2armadilloMatrix3D(matlabOutputMatrix);
                     vTemp.insert(vTemp.end(), aC);
                }
                vDMPTemp.insert(vDMPTemp.end(), vTemp);
                /* test output
                for(vector<cube>::iterator it = vTemp.begin(); it != vTemp.end(); ++it) {
                    (*it).print(cout);
                }
                */
            }
            gmm.Sigma2.insert(gmm.Sigma2.begin(), vDMPTemp.begin(), vDMPTemp.end());
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

void onlineGMR::writeMatlabFile() const
{
    // TODO: Dummy implementation to write to a new matlab file
    MATFile *pmat;

    //create a new mat-file and save some variable/matrix in it
    double dbl1[]={1.1, 4.3, -1.6, -4, -2.75};
    double dbl2[]={-4.9, 2.3, -5};
    mxArray *A, *B;

    A=mxCreateDoubleMatrix(1, 5, mxREAL);
    B=mxCreateDoubleMatrix(1, 3, mxREAL);

    //copy an array to matrix A and B
    memcpy(mxGetPr(A), dbl1, 5 * sizeof(double));
    memcpy(mxGetPr(B), dbl2, 3 * sizeof(double));

    //opening TestVar.mat for writing new data
    pmat=matOpen(outputFile, "w");
    matPutVariable(pmat, "A", A);
    matPutVariable(pmat, "B", B);

    matClose(pmat);

    mxDestroyArray(A);
    mxDestroyArray(B);
}

vec calcPDF(vec X, vec Mu, mat Sigma)
{
    vec diff = X - Mu;
    // TODO add as const?
    double denominator = sqrt(pow(2*M_PI, 3));

    // TODO check exponential function
    return 1 / (denominator * (det(Sigma))) * arma::exp(-1/2 * diff.t() * Sigma.i()* diff);

}

void onlineGMR::regression(vec X_in /* double s, vec T */)
{
    // declare dimensions
    int nDMP = gmm.Priors.size();
    int nDemos = gmm.Priors[0].size();
    int nComponents = gmm.Priors[0][0].n_cols;

    double currFTmp;

    int in[] = {0, 1, 2};
    mat InvSigma2;

    const int DMP = 0;

    vec h[nComponents];

    // calc sum_priors (scalar, vec only because of return type)
    vec sumPriors = 0;
    for (int k=0; k < nComponents; k++) {
        sumPriors += gmm.Priors[DMP][k] * calcPDF(X_in, gmm.Mu[DMP][k], gmm.Sigma2[DMP][k]);
    }

    for (int i=0; i < nDemos; i++) {
        for (int k=0; k < nComponents; k++) {

            // TODO continue here!!!
            //F = gmm.Mu[DMP][k]

            h[k] = as_scalar((gmm.Priors[DMP][k] * calcPDF(X_in, gmm.Mu[DMP][k], gmm.Sigma2[DMP][k])) / sumPriors);
        }
        //InvSigma2 = (gmm.Sigma2[d][in[2]+1][i]).i();
        //currFTmp = gmm.Mu[d][in[2]+1][i] + gmm.Sigma2[d][in[2]+1][i] * InvSigma2 * ([s,h] - gmm.Mu[d][in][i]);

    }


    /*
    for (int d=0; d < nDMP; d++) {
        for (int i=0; i < nDemos; i++) {
            //InvSigma2 = (gmm.Sigma2[d][in[2]+1][i]).i();
            //currFTmp = gmm.Mu[d][in[2]+1][i] + gmm.Sigma2[d][in[2]+1][i] * InvSigma2 * ([s,h] - gmm.Mu[d][in][i]);

        }
    }
    */

    // matlab reference
    // ----------------------------------------------
    /*
    for d=1:nDMP
            for i=1:length(Priors2{d})
              currFTmp = Mu2{d}(in(end)+1,i) + Sigma2{d}(in(end)+1,in,i)*inv(Sigma2{d}(in,in,i)) * ([s;h(:,nb)]-Mu2{d}(in,i));
              rGMR(nb).F(d,n) = rGMR(nb).F(d,n) + rGMR(nb).H(i,n,d) * currFTmp;
            end
        end
    */
}


// calinon reference
// ---------------------------------------------------------
/*
void  GMR::regression(GMM_Model* gmmOut, mat data_in, urowvec in, urowvec out)
{
    // Base regression function. Use this one to be as fast as possible.
    // One need to supply a gmmOut object of appropriate size (nbStates = data_in.n_cols, nbVar = size(out))


    Pxi = zeros(data_in.n_cols,gmm->getNumSTATES());
    //Compute the influence of each GMM component, given input x
    // See Eq. (3.0.5) in doc/TechnicalReport.pdf
    for(i=0; i<gmm->getNumSTATES(); i++){
        Mu = gmm->getMU(i)(in);
        Sigma = gmm->getSIGMA(i)(in,in);
        //Mu_tmp.print("Mu(i) = ");			// DEBUGGING
        //Sigma_tmp.print("Sigma(i) = ");	// DEBUGGING
        Gtmp->setMU(Mu);
        Gtmp->setSIGMA(Sigma);
        Pxi.col(i) = gmm->getPRIORS(i) * (Gtmp->getPDFValue(data_in));
    }
    //Priors.print("Priors = ");	// DEBUGGING
    //data_in->getData().print("t = ");	// DEBUGGING
    //Pxi.print("Pxi = ");	// DEBUGGING
    beta = Pxi / repmat(sum(Pxi,1),1,gmm->getNumSTATES()); // See Eq. (3.0.5) in doc/TechnicalReport.pdf

    //beta.print("beta = ");	// DEBUGGING
    for(t=0; t<data_in.n_cols; t++){
        MuOut.zeros(out.n_elem);
        SigmaOut.zeros(out.n_elem,out.n_elem);
        for(i=0; i<gmm->getNumSTATES(); i++){
            Mu = gmm->getMU(i);
            Sigma = gmm->getSIGMA(i);
            // Pre compute inverse, use inv_sympd to improve performance
            InvSigmaInIn = (Sigma(in,in).i());

            MuOutTmp = Mu(out) + Sigma(out,in) * InvSigmaInIn * (data_in.col(t)-Mu(in)); // See Eq. (3.0.3) in doc/TechnicalReport.pdf
            SigmaOutTmp = Sigma(out,out) - Sigma(out,in) * InvSigmaInIn * Sigma(in,out); // See Eq. (3.0.4) in doc/TechnicalReport.pdf
            MuOut = MuOut + beta(t,i) * MuOutTmp; // See Eq. (3.0.2) in doc/TechnicalReport.pdf
            SigmaOut = SigmaOut + beta(t,i) * SigmaOutTmp; // See Eq. (3.0.2) in doc/TechnicalReport.pdf
        }
//		COMPONENTS.push_back( GaussianDistribution(MuOut, SigmaOut) );
        gmmOut->getCOMPONENTS(t).setMU(MuOut);
        gmmOut->getCOMPONENTS(t).setSIGMA(SigmaOut);

    }

//	GMM_Model* gmmOut;
//	gmmOut->setCOMPONENTS(COMPONENTS);
//	return gmmOut;

}

void GMR::setGMMModel(GMM_Model* gmmmodel)
{
    gmm = gmmmodel;

    // Initial settings for regression
    Gtmp= new GaussianDistribution(gmm->getNumSTATES());
}

} //end of pbdlib namespace

*/



onlineGMR::~onlineGMR()
{

}
