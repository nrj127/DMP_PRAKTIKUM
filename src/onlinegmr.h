#ifndef ONLINEGMR_H
#define ONLINEGMR_H

#include <armadillo>
#include <mat.h>
// #include <matlab.h> to use matlab built in functions

// the matlab library "libmat.lib" has been added to the project in CMakeLists.txt
// in order to read and write matlab files

class onlineGMR
{
private:

    char* inputFile;
    char* outputFile;

public:
    onlineGMR(char *inputFile, char *outputFile);


    void readMatlabFile();

    void writeMatlabFile();

    void regression();





    virtual ~onlineGMR();
};

#endif // ONLINEGMR_H
