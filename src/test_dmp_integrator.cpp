#include <stdlib.h>
#include <math.h>
#include "dmp_integrator.h"
#include "utility.h"

using namespace std;

int main(int argc, char *argv[])
{
    cout << "testing the integrator" << endl;
    dmp_integrator integrator1;

    integrator1.start_integration();
    //utility u1;

    mxArray* s_traj_mat;
    mxArray* x_traj_mat;
    mxArray* v_traj_mat;
    s_traj_mat = mxCreateDoubleMatrix(integrator1.s_traj.size(),integrator1.s_traj[0].size(),mxREAL);
    x_traj_mat = mxCreateDoubleMatrix(integrator1.x_traj.size(),integrator1.x_traj[0].size(),mxREAL);
    v_traj_mat = mxCreateDoubleMatrix(integrator1.v_traj.size(),integrator1.v_traj[0].size(),mxREAL);

    utility::stdVectorMatrix2matlabMatrix(&integrator1.s_traj,s_traj_mat);
    utility::stdVectorMatrix2matlabMatrix(&integrator1.x_traj,x_traj_mat);
    utility::stdVectorMatrix2matlabMatrix(&integrator1.v_traj,v_traj_mat);

    utility::writeMatlabFile(s_traj_mat,"straj","../data/straj.mat");
    utility::writeMatlabFile(x_traj_mat,"xtraj","../data/xtraj.mat");
    utility::writeMatlabFile(v_traj_mat,"vtraj","../data/vtraj.mat");


    //integrator1.save_traj();

    return 0;
}
