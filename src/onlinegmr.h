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

public:
    onlineGMR();

    void readMatlabFile(char* fname);

    void writeMatlabFile(char* fname);

    void regression();





    virtual ~onlineGMR();
};

#endif // ONLINEGMR_H
