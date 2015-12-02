#include "onlinegmr.h"
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

void onlineGMR::armadillo2matlabMatrix(mat *armaMatrix, mxArray *outputMatrix)
{
    int mrows = armaMatrix->n_rows;
    int ncols = armaMatrix->n_cols;
    double *values = armaMatrix->memptr();

    outputMatrix = mxCreateDoubleMatrix(mrows, ncols, mxREAL);
    mxSetPr(outputMatrix, values);
}

void onlineGMR::vector2matlabVector(vector<double> *input, mxArray *outputMatrix)
{
    outputMatrix = mxCreateDoubleMatrix(input->size(), 1, mxREAL);
    mxSetPr(outputMatrix, &(input->at(0)));
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
            gmm.Mu.insert(gmm.Mu.end(), vDMPTemp.begin(), vDMPTemp.end());
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
            gmm.Priors.insert(gmm.Priors.end(), vDMPTemp.begin(), vDMPTemp.end());
        }
        else if (strcmp(name, "Priors_Mixtures") == 0)
        {
            vector< mat > vDMPTemp;
            for (int i=0; i < mxGetN(matlabInputMatrix); i++) {
                mxArray *matlabTempMatrix = mxGetCell(matlabInputMatrix, i);
                mat aM = matlab2armadilloMatrix(matlabTempMatrix);
                vDMPTemp.insert(vDMPTemp.end(), aM);
                /* test output
                for(vector<mat>::iterator it = vTemp.begin(); it != vTemp.end(); ++it) {
                    (*it).print(cout);
                }
                */
            }
            gmm.Priors_mixtures = vDMPTemp;
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

vec onlineGMR::calcPDF(vec X, vec Mu, mat Sigma)
{
    vec diff = X - Mu;
    int dimensions = X.size();
    // TODO add as const?
    double denominator = sqrt(pow(2*M_PI, dimensions));

    // TODO check exponential function
    return 1 / (denominator * (arma::det(Sigma))) * arma::exp(-1/2 * diff.t() * Sigma.i()* diff);
}

mat onlineGMR::regression(vec X_in /* double s, vec T */)
{
    // TODO shift all declarations into members --> faster memory allocation
    // declare dimensions
    int nDMP = gmm.Priors.size();
    int nDemos = gmm.Priors[0].size();
    int kComponents = gmm.Priors[0][0].n_cols;
    int out = gmm.Mu[0][0].n_rows - 1;  // index of output element, usually 3
    int in = out -1;    // last index of input elements, usually 0 ... 2

    vec F(nDMP);
    F.zeros();

    vec currF(kComponents);
    mat InvSigma2(in,in);
    mat sumPriors(nDMP, nDemos);
    sumPriors.zeros();

    vec h(kComponents);

    for (int dmp=0; dmp < nDMP; dmp++) {

        for (int i=0; i < nDemos; i++) {
            for (int k=0; k < kComponents; k++) {
                sumPriors(dmp, i) += as_scalar(gmm.Priors[dmp][i](k) * calcPDF(X_in, gmm.Mu[dmp][i].submat(0,k,in,k), gmm.Sigma2[dmp][i].slice(k).submat(0,0,in,in)));
            }
        }
        for (int i=0; i < nDemos; i++) {
            // TODO nested loops?
            for (int k=0; k < kComponents; k++) {
                // invert Sigma Matrix
                InvSigma2 = gmm.Sigma2[dmp][i].slice(k).submat(0, 0, in, in).i();
                // calc current F term
                currF[k] = as_scalar(gmm.Mu[dmp][i](out, k) + gmm.Sigma2[dmp][i].slice(k).submat(out, 0, out, in) * InvSigma2 * (X_in - gmm.Mu[dmp][i].submat(0,0,in,0)));
                h[k] = as_scalar((gmm.Priors[dmp][i](0, k) * calcPDF(X_in, gmm.Mu[dmp][i].submat(0,0,in,0), gmm.Sigma2[dmp][i].slice(k).submat(0, 0, in, in))) / sumPriors(dmp, i));
            }
            // F[dmp] = sum(h % currF); // % element wise multiplication
            //cout << "F of dmp[" << dmp << "], demo[" << i << "] : " << F[dmp] << endl;
            F[dmp] += gmm.Priors_mixtures[dmp][i] * sum(h % currF);
        }
    }
    // debugForcingTerms(F);

    return F;
}

void onlineGMR::debugForcingTerms(vec F)
{
    cout << "F: " << endl << F << endl;
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
