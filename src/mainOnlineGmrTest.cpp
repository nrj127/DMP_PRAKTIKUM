#include "onlinegmr.h"

int main(int argc, char* argv[]) {

    const char* inputFile = "../data/ModelDMPGaussBetaManyData.mat";
    const char* outputFile = "../data/TestOnlineGmrOutput.mat";

    onlineGMR gmr = onlineGMR(inputFile, outputFile);

    gmr.readMatlabFile();
    //gmr.writeMatlabFile();

    // gmr.regression();


    return 0;
}
