#include "onlinegmr.h"
#include <ctime>

int main(int argc, char* argv[]) {

    /*
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
    */

    // Testing rotation matrix to euler angles
    mat rot;
    rot << -0.7021 << 0.7094 << -0.0618 << endr
        << 0.7114 << 0.7025 << -0.0202 << endr
        << 0.0291 << -0.0581 << -0.9979 << endr;

    vec angles = utility::rotationMatrix2eulerAngles(rot);
    angles.print(cout);
    cout << "--" << endl;

    rot = utility::eulerAngles2rotationMatrix(angles);
    rot.print(cout);


    return 0;
}
