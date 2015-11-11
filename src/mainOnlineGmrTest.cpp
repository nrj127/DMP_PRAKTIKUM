#include "onlinegmr.h"

int main(int argc, char* argv[]) {


    onlineGMR gmr = onlineGMR("../data/ModelDMPGaussBetaManyData.mat", "../data/TestOnlineGmrOutput.mat");

    gmr.readMatlabFile();
    gmr.writeMatlabFile();


    return 0;
}
