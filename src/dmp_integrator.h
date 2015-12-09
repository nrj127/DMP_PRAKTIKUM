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
    static const char* outputFile;
    static const char* gmr_outFile;
    static const char* inputFile;
    void writeMatlabFile(vector<double>& x, vector<double>& v, vector<double>& s);

    static const double tau;
    static const double dt;
    static const double omega_n;
    static const double zeta;
    static const double D;
    double K;           //spring
    static const double alpha;            //decay factor
    static const int nsteps;

    double h_task[2];

    vector<double> v_0;  //starting values
    vector<double> x_0;
    vector<double> s_0;

    static const double g;
    static const double ndmp;

    vector<int> myvector ;


};

#endif // DMP_INTEGRATOR_H
