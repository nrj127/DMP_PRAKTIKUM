#include "dmp_integrator.h"

using namespace dmp_integrator;

dmp_integrator::dmp_integrator()
{

    //this is the main integration loop for the dmp:

    s= s_0;
    v= v_0;
    x= x_0;

    for(int i=0; i<nsteps; i++)
    {
        s += dt/tau*alpha*s;
        v += dt/tau*(K*(g-x) - D*v - K(g-x_0)*s + s*K*F_s);
        x += v/tau;
    }
}
