#include "dmp_integrator.h"

//using namespace dmp_integrator;
using namespace std;
using namespace arma;

const char* dmp_integrator::outputFile="../data/3_dmps_int.mat";
const char* dmp_integrator::gmr_outFile=".";  //not used
const char* dmp_integrator::inputFile = "../data/ModelDMPGaussBetaManyData.mat";

const double dmp_integrator::tau=1;                 //time constant
const double dmp_integrator::dt=.005;               //time step
const double dmp_integrator::omega_n=30;            //natural frequency
const double dmp_integrator::zeta=.707;             //damping ratio
const double dmp_integrator::D=2*zeta*omega_n;      //damping
//double dmp_integrator::K;           //spring
const double dmp_integrator::alpha=.5;              //decay factor
const int dmp_integrator::nsteps= 2000;//2000;

const double dmp_integrator::g=1;     //hardcode that..
const double dmp_integrator::ndmp=3;

dmp_integrator::dmp_integrator()
{
    K=pow(omega_n,2);

    //std::vector<double>::size_type l = nsteps;


    x_traj.resize( ndmp , vector<double>( nsteps , 0.0 ) );
    v_traj.resize( ndmp , vector<double>( nsteps , 0.0 ) );
    s_traj.resize( ndmp , vector<double>( nsteps , 0.0 ) );

    v_0.resize(ndmp,0);
    s_0.resize(ndmp,1);

    // end pose
    x_0[0] = -0.521055939000000;
    x_0[1] = -0.285697768600000;
    x_0[2] = 2.33119239481157;

    //end pose
    g[0] = -0.518771839625202;
    g[1] = 0.241740713536797;
    g[2] = 2.36155119110486;


    h_task[0] = -0.4105;
    h_task[1] =  0.0552;

    /*
    x_traj.insert(x_traj.begin(),l,0.0);   //initialize vectors with zeros
    v_traj.insert(v_traj.begin(),l,0.0);
    s_traj.insert(s_traj.begin(),l,0.0);
    */
}

void dmp_integrator::start_integration()
{
    //this is the main integration loop for the dmp:
    double s,v,x;
    cout << "size of vector s:  " << s_traj.size() <<endl;
    cout << "size of capacity s:  " << s_traj.capacity() <<endl;
    for (int i =0; i< 3; i++)
        s_traj[i][0]=s_0[i];

    for (int i =0; i< 3; i++)
        v_traj[i][0]=v_0[i];

    for (int i =0; i< 3; i++)
        x_traj[i][0]=x_0[i];

    onlineGMR gmr = onlineGMR(inputFile, outputFile);

    gmr.readMatlabFile();

    vec X_in(3);

    for(int i=0; i<nsteps-1; i++)
    {
        for(int j=0; j < ndmp; j++)
        {

            s=s_traj[j][i];
            v=v_traj[j][i];
            x=x_traj[j][i];

            vector<double> F_vec;
            X_in[0]=s;
            X_in[1]=h_task[0];
            X_in[2]=h_task[1];
            F_vec=gmr.regression(X_in);
            int F = F_vec[j];

            //F=0; // for testing!

            s_traj[j][i+1] = -dt/tau*alpha*s + s;
            v_traj[j][i+1] = dt/tau*(K*(g[j]-x) - D*v - K*(g[j]-x_0[j])*s + s*K*F) + v;
            x_traj[j][i+1] = dt/tau*v + x;

            /*
            cout << "x: " << x << endl;
            cout << "v: " << v << endl;
            cout << "s: " << s << endl;
            */
        }
    }
    cout << "here" << endl;
    //save to .mat files
}

