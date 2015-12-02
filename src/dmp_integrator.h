#ifndef DMP_INTEGRATOR_H
#define DMP_INTEGRATOR_H

#include <armadillo>
#include <iostream>
#include <mat.h>
#include <matrix.h>
#include <vector>
#include <armadillo>


using namespace std;

class dmp_integrator
{
public:
    dmp_integrator();

    void start_integration(void);
    void save_traj();

private:
    char* outputFile="../data/3_dmps_int.mat";
    char* gmr_outFile=".";  //not used
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


    const vector<double> v_0={0,0,0}; //starting values
    const vector<double> x_0={0,0,0};
    const vector<double> s_0={1,1,1};

    const double g=1;     //hardcode that..
    const double F=0;
    const double ndmp=3;

    vector<int> myvector ;

    vector< vector<double> > x_traj;
    vector< vector<double> > v_traj;
    vector< vector<double> > s_traj;
};

#endif // DMP_INTEGRATOR_H
