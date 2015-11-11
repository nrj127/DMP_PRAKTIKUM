#include <stdlib.h>
#include <math.h>
#include "dmp_integrator.h"

using namespace std;

int main(int argc, char *argv[])
{
    cout << "testing the integrator" << endl;
    dmp_integrator integrator1;

    integrator1.start_integration();
    integrator1.save_traj();

    return 0;
}
