/**
 
 
Copyright (C) 2015, Martijn Zeestraten 

This file is part of PbDLib.

PbDLib is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

PbDLib is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with PbDLib.  If not, see <http://www.gnu.org/licenses/>.
*/

/*! \file 
\brief HMM_model class
The class HSMM_model allows to use a Gaussian Mixture Model and to learn the parameters from task demonstrations

*/

#ifndef HSMM_H
#define HSMM_H

#include "pbdlib/datapoints.h"
#include "pbdlib/hmm.h"
#include "pbdlib/demonstration.h"
#include "pbdlib/mvn.h"
#include "armadillo"
#include <math.h>

using namespace arma;

namespace pbdlib 
{

class HSMM:public HMM
{
	private:
		// HMM Components:
		std::vector<GaussianDistribution> DurationCOMPONENTS;

		// Variables for calculating Forward Variable:
		GaussianDistribution* Gtmp;
		mat Atmp1, Atmp2; // Help variables for the calculation of alpha variables
		mat    ALPHA;     // Variable to keep track of past alphas
		colvec bmx;       // variable used to keep track of observation probability P(x|model)
		colvec btmp;      // Unnormalized observation probabilities
		colvec S;         // 

		mat    Pd;        // Matrix with precomputed probabilty values for the duration;
		uint PdSize;
		bool Initialized,tmpInit;   // Variables used to check if alpha calculation is initialized

		// For predictions:
		colvec alphatmp;  // Temp variable to hold in the loop alpha
		mat ALPHAtmp;  // to store ALPHA in the prediction loop
		mat    AlphaPred; // Matrix to hold predictions
		colvec Stmp;      // to store S in the prediction loop

		void initializeFwdCalculation();
		void updateBtmp(colvec& _Btmp,colvec& _obs);
		void updateBtmp(colvec& _Btmp,colvec& _obs,urowvec& _ind);
		void updateBmx(colvec& _bmx, mat& ALPHA, colvec& _btmp);
		void updateALPHA(mat& ALPHA, colvec& S, colvec& _bmx);
		void updateALPHA(mat& ALPHA, colvec& S);
		void updateS(colvec& _S, mat& ALPHA, colvec& _bmx);
		void updateS(colvec& _S, mat& ALPHA);
		void updateAlpha(colvec& _alpha, mat& ALPHA, colvec& btmp);
		void updateAlpha(colvec& _alpha, mat& ALPHA);
	
	
		// Forward variable Functions
		void initializeForwardVariable(); // initialization without observation
		void initializeForwardVariable(colvec& obs); // initialization with observation
		void initializeForwardVariable(colvec& obs, urowvec& ind); // initializaiton with observation
		
		void lstepForwardVariable(); // Forward step without observation
		void lstepForwardVariable(colvec& obs); // Forward step with observation
		void lstepForwardVariable(colvec& obs,urowvec& ind); // Forward step with partial observation
		
		colvec alpha;     // Forward variable
	public:
		HSMM(uint _nSTATES, uint _nVARS);		
		HSMM(const std::string &priors_path,
			   const std::string &mu_path, 
			   const std::string &sigma_path,
			   const std::string &transition_path, 
			   const std::string &durMu_path, 
			   const std::string &durSigma_path);
		~HSMM(){}
		
		// Properties:
		std::vector<GaussianDistribution>& getDurationCOMPONENTS();
		GaussianDistribution& getDurationCOMPONENTS(uint);
		colvec& getDurMU(uint);
		mat& getDurSIGMA(uint);

		// Forward Variables:
		void stepForwardVariable(); // Forward step without observation
		void stepForwardVariable(colvec& obs); // Forward step with observation
		void stepForwardVariable(colvec& obs,urowvec& ind); // Forward step with partial observation
		colvec& getForwardVariable(){return alpha;} // Get current forward variable
		void resetForwardVariable(){Initialized=false;}
		
		mat& predictForwardVariable(uint N); // Predict forward variable without predicted observations
		void predictForwardVariable(mat& _AlphaPred); // Predict forward variable without predicted observations (implementation for real-time)



		void setDurationCOMPONENTS(const std::vector<GaussianDistribution>& components);
		void setDurMU(int, const colvec&);
		void setDurSIGMA(int, const mat&);

	};

} // PbdLib namespace

#endif

