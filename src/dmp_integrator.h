#ifndef DMP_INTEGRATOR_H
#define DMP_INTEGRATOR_H

#include <armadillo>
#include <iostream>
#include <mat.h>
#include <matrix.h>
#include <vector>
#include <armadillo>
#include "onlinegmr.h"


using namespace std;

class dmp_integrator
{
public:
    dmp_integrator();

    void start_integration(void);
    vector< vector<double> > x_traj;
    vector< vector<double> > v_traj;
    vector< vector<double> > s_traj;

private:
    const char* outputFile="../data/3_dmps_int.mat";
    const char* gmr_outFile=".";  //not used
    const char* inputFile = "../data/ModelDMPGaussBetaManyData.mat";
    void writeMatlabFile(vector<double>& x, vector<double>& v, vector<double>& s);

    const double tau=1;                 //time constant
    const double dt=.005;               //time step
    const double omega_n=30;            //natural frequency
    const double zeta=.707;             //damping ratio
    const double D=2*zeta*omega_n;      //damping
    double K;           //spring
    const double alpha=.5;              //decay factor
    const int nsteps= 2000;//2000;

    double h_task[2];

    vector<double> v_0;  //starting values
    vector<double> x_0;
    vector<double> s_0;

    const double g=1;     //hardcode that..
    const double ndmp=3;

    vector<int> myvector ;


};

#endif // DMP_INTEGRATOR_H
