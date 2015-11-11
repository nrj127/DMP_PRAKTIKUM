#include "dmp_integrator.h"


//using namespace dmp_integrator;
using namespace std;
using namespace arma;

dmp_integrator::dmp_integrator()
{
    K=pow(omega_n,2);
}

void dmp_integrator::start_integration()
{
    //this is the main integration loop for the dmp:

    s= s_0;
    v= v_0;
    x= x_0;

    for(int i=0; i<nsteps; i++)
    {
        s += dt/tau*alpha*s;
        v += dt/tau*(K*(g-x) - D*v - K*(g-x_0)*s + s*K);
        x += v/tau;

        s_traj[i]=s;
        x_traj[i]=x;
        s_traj[i]=s;

        cout << "x" << x << endl;
        cout << "v" << v << endl;
        cout << "s" << s << endl;
    }

    //save to .mat files
}

void dmp_integrator::writeMatlabFile(double* x, int xl, double* s, int sl,double* v, int vl)
{
    // TODO: Dummy implementation to write to a new matlab file


    MATFile *pmat;

    //create a new mat-file and save some variable/matrix in it
    double dbl1[]={1.1, 4.3, -1.6, -4, -2.75};
    double dbl2[]={-4.9, 2.3, -5};
    mxArray *X, *S, *V;

    X=mxCreateDoubleMatrix(1, xl, mxREAL);
    S=mxCreateDoubleMatrix(1, sl, mxREAL);
    V=mxCreateDoubleMatrix(1, vl, mxREAL);

    //copy an array to matrix A and B
    memcpy(mxGetPr(X), x, xl * sizeof(double));
    memcpy(mxGetPr(S), s, sl * sizeof(double));
    memcpy(mxGetPr(V), v, vl * sizeof(double));

    //opening TestVar.mat for writing new data
    pmat=matOpen(outputFile, "w");
    matPutVariable(pmat, "X", X);
    matPutVariable(pmat, "S", S);
    matPutVariable(pmat, "V", V);

    matClose(pmat);

    mxDestroyArray(X);
    mxDestroyArray(S);
    mxDestroyArray(V);
}

void dmp_integrator::save_traj()
{
    writeMatlabFile(this->x_traj,this->nsteps,this->s_traj,this->nsteps,this->v_traj,this->nsteps);
}

