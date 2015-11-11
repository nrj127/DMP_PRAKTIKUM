#ifndef ONLINEGMR_H
#define ONLINEGMR_H

#include <armadillo>
#include <mat.h>
#include <matrix.h>

// #include <matlab.h> to use matlab built in functions

// the matlab library "libmat.lib" has been added to the project in CMakeLists.txt
// in order to read and write matlab files

using namespace std;
using namespace arma;

class onlineGMR
{
private:

    char* inputFile;
    char* outputFile;

    mat armadilloMatrix(mxArray *matlabMatrix);
public:
    onlineGMR(char *inputFile, char *outputFile);


    void readMatlabFile();

    void writeMatlabFile();

    void regression();





    virtual ~onlineGMR();
};

#endif // ONLINEGMR_H
