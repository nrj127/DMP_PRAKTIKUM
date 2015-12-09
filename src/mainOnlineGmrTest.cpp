#include "onlinegmr.h"
#include <ctime>

int main(int argc, char* argv[]) {

    const char* inputFile = "../data/ModelDMPGaussBetaManyData.mat";
    const char* outputFile = "../data/TestOnlineGmrOutput.mat";

    onlineGMR gmr = onlineGMR(inputFile, outputFile);

    gmr.readMatlabFile();

    vec X_in(3);
    X_in[0] = 1;
    X_in[1] = 1;
    X_in[2] = 1;

    vec F(3);

    // test pdf
    vec X(1);
    vec Mu(1);  Mu[0] = 0;
    mat Sigma(1,1);    Sigma(0,0) = 1;
    vec out(1);

    for (int i=0; i<100; i++) {
        X[0] = i/10.0;
        out = gmr.calcPDF(X, Mu, Sigma);
        out.print(cout, "");
    }

    while (X_in(0) >= 0) {

        X_in(0) -= 0.1;

        // start time measurement
        clock_t begin = clock();
        F = gmr.regression(X_in);

        cout << F(2) << ' ';

        // finish time measurement
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        cout << "Elapsed time (in ms) : " << elapsed_secs * 1000.0 << endl;
    }

    //gmr.writeMatlabFile();

    return 0;
}
