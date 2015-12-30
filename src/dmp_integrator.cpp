#include "dmp_integrator.h"

//using namespace dmp_integrator;
using namespace std;
using namespace arma;

const char* dmp_integrator::outputFile="../data/3_dmps_int.mat";
const char* dmp_integrator::gmr_outFile=".";  //not used
const char* dmp_integrator::inputFile = "../data/ModelDMPGaussBetaManyData3.mat";

const double dmp_integrator::tau=1;                 //time constant
const double dmp_integrator::dt=.005;               //time step
const double dmp_integrator::omega_n=30;            //natural frequency
const double dmp_integrator::zeta=.707;             //damping ratio
const double dmp_integrator::D=2*zeta*omega_n;      //damping
//double dmp_integrator::K;           //spring
const double dmp_integrator::alpha=.5;              //decay factor
const int dmp_integrator::nsteps= 2100;//2000;

dmp_integrator::dmp_integrator() : gmr(inputFile, outputFile)
{
    K=pow(omega_n,2.0);



    //std::vector<double>::size_type l = nsteps;
    cout << "out1" << endl;

    x_traj.resize( ndmp , vector<double>( nsteps , 0.0 ) );
    v_traj.resize( ndmp , vector<double>( nsteps , 0.0 ) );
    s_traj.resize( ndmp , vector<double>( nsteps , 0.0 ) );

    cout << "out2" << endl;
    v_0.resize(ndmp,0);
    s_0.resize(ndmp,1);

    x_0.resize(3);
    // start pose
    x_0[0] = -0.521055939000000;
    x_0[1] = -0.285697768600000;
    x_0[2] = 2.33119239481157;

    g.resize(3);
    //end pose
    g[0] = -0.518771839625202;
    g[1] = 0.241740713536797;
    g[2] = 2.36155119110486;

    // task parameters
    //h_task[0] = -0.4105; //orig
    // h_task[1] =  0.0552; //orig

    //h_task[0] = -0.3;
    //h_task[1] =  0.04;
    //h_task[0] = -0.6;
    //h_task[1] =  -0.04;

    /*
    x_traj.insert(x_traj.begin(),l,0.0);   //initialize vectors with zeros
    v_traj.insert(v_traj.begin(),l,0.0);
    s_traj.insert(s_traj.begin(),l,0.0);
    */

    cout << "out3" << endl;


    for (int i =0; i< 3; i++)
        s_traj[i][0]=s_0[i];

    for (int i =0; i< 3; i++)
        v_traj[i][0]=v_0[i];

    for (int i =0; i< 3; i++)
        x_traj[i][0]=x_0[i];

    iteration = 0;  // starting at iteration 0

    cout << "out3" << endl;

    //gmr = onlineGMR(inputFile, outputFile);
    gmr.readMatlabFile();

    // get number of DMPs
    ndmp = gmr.getnDMP();
}

void dmp_integrator::start_integration(vec TaskParams)
{
    //this is the main integration loop for the dmp:
    double s,v,x,F;

    vec X_in(3);
    vector<double> F_vec;

    for(int i=0; i<nsteps-1; i++)
    {
        X_in[0]=s_traj[0][i];
        // old: X_in[1]=h_task[0];
        // old: X_in[2]=h_task[1];

        // Write task params into X_in vector
        X_in.cols(1,X_in.n_elem-1) = TaskParams;


        F_vec=gmr.regression(X_in);

        for(int j=0; j < ndmp; j++)
        {

            s=s_traj[0][i];
            v=v_traj[j][i];
            x=x_traj[j][i];

            F = F_vec[j];

            s_traj[j][i+1] = -dt/tau*alpha*s + s;
            v_traj[j][i+1] = dt/tau*(K*(g[j]-x) - D*v - K*(g[j]-x_0[j])*s + s*K*F) + v;

            x_traj[j][i+1] = dt/tau*v + x;
        }
    }
}

vector<double> dmp_integrator::integrate_onestep(vec TaskParams)
{
    double s,v,x,F;

    vec X_in( TaskParams.n_elem + 1 );
    vector<double> F_vec;
    vector<double> new_x(3);

    int i= iteration;
    iteration ++;

    X_in[0]=s_traj[0][i];
    // old: X_in[1]=h_task[0];
    // old: X_in[2]=h_task[1];

    // Write task params into X_in vector
    X_in.cols(1,X_in.n_elem-1) = TaskParams;

    F_vec=gmr.regression(X_in);

    for(int j=0; j < ndmp; j++)
    {

        s=s_traj[0][i];
        v=v_traj[j][i];
        x=x_traj[j][i];

        F = F_vec[j];

        s_traj[j][i+1] = -dt/tau*alpha*s + s;
        //cout << "iteration" << iteration << "s:" << s << endl;
        v_traj[j][i+1] = dt/tau*(K*(g[j]-x) - D*v - K*(g[j]-x_0[j])*s + s*K*F) + v;

        x_traj[j][i+1] = dt/tau*v + x;

        new_x.at(j)=x_traj[j][i+1];  //writing to results
    }
    return new_x;
}


// destructor
dmp_integrator::~dmp_integrator()
{
    cout << "dmp_integrator: destructor frees memory" << endl;
}

