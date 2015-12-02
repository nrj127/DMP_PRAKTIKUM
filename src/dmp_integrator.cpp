#include "dmp_integrator.h"
#include "onlinegmr.h"

//using namespace dmp_integrator;
using namespace std;
using namespace arma;

dmp_integrator::dmp_integrator()
{
    K=pow(omega_n,2);

    std::vector<double>::size_type l = nsteps;

    x_traj.insert(x_traj.begin(),l,0.0);   //initialize vectors with zeros
    v_traj.insert(v_traj.begin(),l,0.0);
    s_traj.insert(s_traj.begin(),l,0.0);

}

void dmp_integrator::start_integration()
{
    //this is the main integration loop for the dmp:

    cout << "size of vector s:  " << s_traj.size() <<endl;
    cout << "size of capacity s:  " << s_traj.capacity() <<endl;
    s_traj.at(0)=s_0;
    v_traj.at(0)=v_0;
    x_traj.at(0)=x_0;

    onlineGMR gmr = onlineGMR(inputFile, outputFile);

    for(int i=0; i<nsteps-1; i++)
    {
        s=s_traj.at(i);
        v=v_traj.at(i);
        x=x_traj.at(i);


        F=gmr(s,taskp_vector)

        s_traj.at(i+1) = -dt/tau*alpha*s + s;
        v_traj.at(i+1) = dt/tau*(K*(g-x) - D*v - K*(g-x_0)*s + s*K*F) + v;
        x_traj.at(i+1) = dt/tau*v + x;

        cout << "x: " << x << endl;
        cout << "v: " << v << endl;
        cout << "s: " << s << endl;
    }

    //save to .mat files
}

void dmp_integrator::writeMatlabFile(std::vector<double>& x, std::vector<double>& s, std::vector<double>& v)
{
    // TODO: Dummy implementation to write to a new matlab file


    MATFile *pmat;

    //create a new mat-file and save some variable/matrix in it
    mxArray *X, *S, *V;

    int xl = x.size();
    int sl = s.size();
    int vl = v.size();

    X=mxCreateDoubleMatrix(1, xl, mxREAL);
    S=mxCreateDoubleMatrix(1, sl, mxREAL);
    V=mxCreateDoubleMatrix(1, vl, mxREAL);

    //copy an array to matrix A and B
    memcpy(mxGetPr(X), &x[0], xl * sizeof(double));
    memcpy(mxGetPr(S), &s[0], sl * sizeof(double));
    memcpy(mxGetPr(V), &v[0], vl * sizeof(double));

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
    writeMatlabFile(x_traj,s_traj,v_traj);
}

