#ifndef DMP_INTEGRATOR_H
#define DMP_INTEGRATOR_H

#include <armadillo>
#include <iostream>
#include <mat.h>
#include <matrix.h>
#include <vector>

class dmp_integrator
{
public:
    dmp_integrator();

    void start_integration(void);
    void save_traj();

private:
    char* outputFile="../data/dmp_test.mat";
    void writeMatlabFile(std::vector<double>& x, std::vector<double>& v, std::vector<double>& s);

    const double tau=1;                 //time constant
    const double dt=.005;               //time step
    const double omega_n=30;            //natural frequency
    const double zeta=.707;             //damping ratio
    const double D=2*zeta*omega_n;      //damping
    double K;           //spring
    const double alpha=.5;              //decay factor
    const double s_0=1;                 //start value for s
    double s;                     //time variable
    const int nsteps= 2000;//2000;
    double v;
    const double v_0=0;
    double x;
    const double x_0=0;
    const double g=1;     //hardcode that..
    const double F=0;

    std::vector<int> myvector ;

    std::vector<double> x_traj;
    std::vector<double> v_traj;
    std::vector<double> s_traj;
};

#endif // DMP_INTEGRATOR_H
