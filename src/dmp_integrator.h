#ifndef DMP_INTEGRATOR_H
#define DMP_INTEGRATOR_H

#include <armadillo>
#include <iostream>
#include <mat.h>
#include <matrix.h>
#include <vector>
#include "onlinegmr.h"


using namespace std;

class dmp_integrator
{
public:
    dmp_integrator();

    void init();   // read in matlab files and some other initialization
    void start_integration(vec TaskParams);        // for testing full trajectory calculation
    vector<double> integrate_onestep(vec TaskParams);  // used in the control loop

    vector< vector<double> > x_traj;
    vector< vector<double> > v_traj;
    vector< vector<double> > s_traj;

    int iteration;  //the current iteratio

    virtual ~dmp_integrator();

    vector<double> getx_0();


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

    // old: double h_task[2];
    vector<double> h_task;

    vector<double> v_0;  //starting values
    vector<double> x_0;
    vector<double> s_0;

    int ndmp;

    vector<double> g;     //end poses

    vector<int> myvector ;
    onlineGMR gmr;



};

#endif // DMP_INTEGRATOR_H
