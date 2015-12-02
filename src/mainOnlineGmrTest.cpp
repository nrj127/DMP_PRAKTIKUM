#include "onlinegmr.h"
#include <ctime>

int main(int argc, char* argv[]) {

    const char* inputFile = "../data/ModelDMPGaussBetaManyData.mat";
    const char* outputFile = "../data/TestOnlineGmrOutput.mat";

    onlineGMR gmr = onlineGMR(inputFile, outputFile);

    gmr.readMatlabFile();

    vec X_in(3);
    X_in[0] = 1;
    X_in[1] = 2;
    X_in[2] = 3;

    // start time measurement
    clock_t begin = clock();

    gmr.regression(X_in);

    // finish time measurement
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Elapsed time for regression function (in ms) : " << elapsed_secs * 1000.0 << endl;

    //gmr.writeMatlabFile();

    return 0;
}
