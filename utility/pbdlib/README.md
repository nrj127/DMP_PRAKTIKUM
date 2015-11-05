# PbDLib C++ Library

Copyright (C) 2015, Davide De Tommaso, Leonel Rozo, Tohid Alizadeh, Milad Malekzadeh, João Silvério, Sylvain Calinon, Danilo Bruno, Martijn Zeestraten, Ioannis Havoutis and Daniel Berio. Pbdlib has been jointly developed at the Italian Institute of Technology, Genoa, Italy and at the Idiap Research Institute, Martigny, Switzerland, under a GPL Version 3 license.

See also https://gitlab.idiap.ch/rli/pbdlib-matlab for a Matlab/GNU-Octave version of the library containing additional experimental functionalities.

### Dependencies

PbDLib requires Armadillo for linear algebra operations, which runs faster if Lapack is also installed. PbDLib can be compiled with CMake.

```
sudo apt-get install cmake liblapack3 liblapack-dev libarmadillo4 libarmadillo-dev
```
 
### Installation instructions

```
cd pbdlib
mkdir build
cd build
cmake ..
make
sudo make install
```

A GUI can be used after installation of the library: 
https://gitlab.idiap.ch/rli/pbdlib_gui

### Test (assuming a build folder was created for cmake install)

```
cd examples
./test_gmm
```
