#ifndef DMP_INTEGRATOR_H
#define DMP_INTEGRATOR_H

class dmp_integrator
{
public:
    dmp_integrator();

private:
    const double tau=1;                 //time constant
    const double dt=.005;               //time step
    const double omega_n=30;            //natural frequency
    const double zeta=.707;             //damping ratio
    const double D=2*zeta*omega_n;      //damping
    const double K=omega_n^2;           //spring
    const double alpha=.5;              //decay factor
    const double s_0=1;                 //start value for s
    const double s;                     //time variable
    const int nsteps=2000;
    const double v;
    const double v_0=0;
    const double x;
    const double x_0=0;
    const double g;     //hardcode that..


};

#endif // DMP_INTEGRATOR_H
