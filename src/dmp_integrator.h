#ifndef DMP_INTEGRATOR_H
#define DMP_INTEGRATOR_H

#include <armadillo>
#include <iostream>
#include <mat.h>
#include <matrix.h>

class dmp_integrator
{
public:
    dmp_integrator();

    void start_integration(void);
    void save_traj();

private:
    char* outputFile="../data/dmp_test.mat";
    void writeMatlabFile(double* x, int xl, double* s, int sl,double* v, int vl);

    const double tau=1;                 //time constant
    const double dt=.005;               //time step
    const double omega_n=30;            //natural frequency
    const double zeta=.707;             //damping ratio
    const double D=2*zeta*omega_n;      //damping
    double K;           //spring
    const double alpha=.5;              //decay factor
    const double s_0=1;                 //start value for s
    double s;                     //time variable
    const int nsteps= 20;//2000;
    double v;
    const double v_0=0;
    double x;
    const double x_0=0;
    const double g=1;     //hardcode that..

    double x_traj[20];
    double v_traj[20];
    double s_traj[20];

};

#endif // DMP_INTEGRATOR_H
