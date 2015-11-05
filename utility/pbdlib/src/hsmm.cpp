/**
Copyright (C) 2015,	Martijn Zeestraten 

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

#include "pbdlib/hsmm.h"

namespace pbdlib
{

HSMM::HSMM(const std::string &priors_path, 
			const std::string &mu_path, 
			const std::string &sigma_path, 
			const std::string &transition_path, 
			const std::string &durMu_path, 
			const std::string &durSigma_path):
	HMM(priors_path,mu_path,sigma_path,transition_path)
{
	mat durMu, durSigma;
	
	// Load components
	durMu.load(durMu_path, raw_ascii);
	durSigma.load(durSigma_path, raw_ascii);

	// Set components:
	mat _SIGMA = zeros(1,1);
	colvec _MU = zeros(1,1);
	std::vector<GaussianDistribution> components;

	for(uint i=0; i<this->getNumSTATES(); i++){
		_MU(0,0) = durMu(0,i);
		_SIGMA(0,0) = durSigma(0,i);
		// for debugging
		components.push_back(GaussianDistribution(_MU, _SIGMA));
	}
	setDurationCOMPONENTS(components);

	// Forward variable calculation initialization:
	initializeFwdCalculation();
}


void HSMM::setDurationCOMPONENTS(const std::vector<GaussianDistribution>& components)
{
	if(components.size() == this->getNumSTATES())
		DurationCOMPONENTS = components;
	else
		std::cout << "\n [ERROR]::HMM::setCOMPONENTS if(components.size() == nSTATES) ... else .";
}

void HSMM::setDurMU(int id, const colvec& _MU)
{
	DurationCOMPONENTS[id].setMU(_MU);
}

void HSMM::setDurSIGMA(int id, const mat& _SIGMA)
{
	DurationCOMPONENTS[id].setSIGMA(_SIGMA);
}


std::vector<GaussianDistribution>& HSMM::getDurationCOMPONENTS()
{
	return DurationCOMPONENTS;
}

GaussianDistribution& HSMM::getDurationCOMPONENTS(uint id)
{
	return DurationCOMPONENTS[id];
}

colvec& HSMM::getDurMU(uint id)
{
	return DurationCOMPONENTS[id].getMU();
}

mat& HSMM::getDurSIGMA(uint id)
{
	return DurationCOMPONENTS[id].getSIGMA();
}

void HSMM::initializeFwdCalculation()
{
	// Calculate PD size:
	cout << "Pd size : ";
	PdSize=0;
	for (uint i = 0;i<this->getNumSTATES();i++)
	{
		if (PdSize <accu(this->getDurMU(i)))
			PdSize = accu(this->getDurMU(i));
	}
	PdSize *=1.5; // Enlarge the Size of Pd for 'accuracy'
	//PdSize = 50;
	cout << PdSize << endl;

	// This function is used to set the forward variable matrices to the appropriate size:
	ALPHA.set_size(this->getNumSTATES(), PdSize);
	Atmp1.set_size(this->getNumSTATES(),PdSize-1);
	Atmp2.set_size(this->getNumSTATES(),PdSize-1);
	alpha.set_size(this->getNumSTATES());
	bmx.set_size(this->getNumSTATES());
	btmp.set_size(this->getNumSTATES());
	S.set_size(this->getNumSTATES());
	Pd.set_size(this->getNumSTATES(),PdSize);
	
	// Duration input vector
	mat dur = linspace(1,PdSize, PdSize);
	// Pre-Calculate Pd
	for (uint i = 0;i<this->getNumSTATES();i++)
	{
		// Note that we need to transpose twice....
		// getPDFValue accepts a rowvec of values, but returns a colvec....
		Pd.row(i) = getDurationCOMPONENTS(i).getPDFValue(dur.t()).t();

	}

	// For forward variable:
	Gtmp = new GaussianDistribution(this->getNumVARS());

	Initialized = false;
}


// -------------------------------------------------------------------
/* 				Forward Variable Step calculations 					*/
// -------------------------------------------------------------------

void HSMM::stepForwardVariable()
{
	if (Initialized==false)
	{
		initializeForwardVariable();
		Initialized = true;
	}
	else
	{
		lstepForwardVariable();
	}
}

void HSMM::stepForwardVariable(colvec& obs)
{
	if (Initialized==false)
	{
		initializeForwardVariable(obs);
		Initialized = true;
	}
	else
	{
		lstepForwardVariable(obs);
	}
}

void HSMM::stepForwardVariable(colvec& obs, urowvec& ind)
{
	if (Initialized==false)
	{
		initializeForwardVariable(obs, ind);
		Initialized = true;
	}
	else
	{
		lstepForwardVariable(obs, ind);
	}
}
// -------------------------------------------------------------------
// ----------  INITIALIZATION of Forward Variable
void  HSMM::initializeForwardVariable()
{
	// ALPHA Variable
	ALPHA = Pd;
	ALPHA.each_col() %= this->getPRIORS().t(); // % is the element wise product

	// Update S
	updateS(S,ALPHA);

	// Update Alpha
	updateAlpha(alpha,ALPHA);
}

void HSMM::initializeForwardVariable(colvec& _obs)
{
	// Calculate the initial forward step
	updateBtmp(btmp,_obs);

	// ALPHA Variable
	ALPHA = Pd;
	ALPHA.each_col() %= this->getPRIORS().t(); // % is the element wise product

	// Update bmx:
	updateBmx(bmx, ALPHA, btmp);

	// Update S
	updateS(S,ALPHA,bmx);

	// Update Alpha
	updateAlpha(alpha,ALPHA,btmp);
}


void HSMM::initializeForwardVariable(colvec& _obs, urowvec& _ind)
{
	// Calculate the initial forward step
	updateBtmp(btmp,_obs, _ind);
	
	// ALPHA Variable
	ALPHA = Pd;
	ALPHA.each_col() %= this->getPRIORS().t(); // % is the element wise product

	// Update bmx:
	updateBmx(bmx, ALPHA, btmp);

	// Update S
	updateS(S,ALPHA,bmx);

	// Update Alpha
	updateAlpha(alpha,ALPHA,btmp);
}


// -------------------------------------------------------------------
// ---------- Step functions of Forward Variable
void HSMM::lstepForwardVariable()
{
	// No observation, assume all btmp is 1 (i.e. observation is equally likeli for all
	// states	
	
	// ALPHA Variable
//	cout << "ALPHA: " << endl;
	updateALPHA(ALPHA,S);
//	cout << ALPHA.t() << endl;

	// Update S
//	cout << "S   : ";
	updateS(S,ALPHA);
//	cout << S << endl;

	// Update Alpha
//	cout << "alpha : " ;
	updateAlpha(alpha,ALPHA);
//	cout << alpha.t() << endl;
}

void HSMM::lstepForwardVariable(colvec& _obs)
{
	// update Btmp:
	updateBtmp(btmp,_obs);
	//cout << "Btmp: " << endl << btmp << endl;
	
	// ALPHA Variable
	updateALPHA(ALPHA,S);

	// Update bmx:
	updateBmx(bmx, ALPHA, btmp);
	//cout << "Bmx: " << endl << bmx << endl;

	// Update S
	updateS(S,ALPHA,bmx);

	// Update Alpha
	updateAlpha(alpha,ALPHA,btmp);
	//cout << "Alpha: " << endl << alpha << endl;
}

void HSMM::lstepForwardVariable(colvec& _obs, urowvec& _ind)
{
	// update Btmp:
	updateBtmp(btmp,_obs,_ind);
	
	// ALPHA Variable
	updateALPHA(ALPHA,S);

	// Update bmx:
	updateBmx(bmx, ALPHA, btmp);

	// Update S
	updateS(S,ALPHA,bmx);

	// Update Alpha
	updateAlpha(alpha,ALPHA,btmp);
}



// -------------------------------------------------------------------
// ---------- Prediction Functions for Forward Variable
mat& HSMM::predictForwardVariable(uint _N)
{
	//cout << "Allocating Memory: ";
	AlphaPred.set_size(this->getNumSTATES(), _N);
	//cout << AlphaPred.n_rows << " x " << AlphaPred.n_cols << endl;
	
	tmpInit = Initialized;
	// Make copy of the current state of the system:
	ALPHAtmp = ALPHA;
	Stmp = S;

	
	for (uint i = 0;i<_N;i++)
	{
		// Alpha variable
		if (tmpInit==false)
		{
			// Initialize: 
			ALPHAtmp = Pd;
			ALPHAtmp.each_col() %= this->getPRIORS().t(); // % is the element wise product
			tmpInit = true;
		}
		else
		{
			updateALPHA(ALPHAtmp,Stmp);
		}	

		// Update S
		updateS(Stmp,ALPHAtmp);

		updateAlpha(alphatmp,ALPHAtmp);

		// Save alpha
		AlphaPred.col(i) = alphatmp;
	}
	return AlphaPred;
}

// Implementation for real-time state prediction (no checks on sizes done)
// We assume that _AlphaPred = nbStates x nbPred of size
void HSMM::predictForwardVariable(mat& _AlphaPred)
{
	
	tmpInit = Initialized;
	// Make copy of the current state of the system:
	ALPHAtmp = ALPHA;
	Stmp = S;

	
	for (uint i = 0;i<_AlphaPred.n_cols;i++)
	{
		// Alpha variable
		if (tmpInit==false)
		{
			// Initialize: 
			ALPHAtmp = Pd;
			ALPHAtmp.each_col() %= this->getPRIORS().t(); // % is the element wise product
			tmpInit = true;
		}
		else
		{
			updateALPHA(ALPHAtmp,Stmp);
		}	

		// Update S
		updateS(Stmp,ALPHAtmp);

		// Update Alpha
		updateAlpha(alphatmp,ALPHAtmp);

		// Save alpha
		_AlphaPred.col(i) = alphatmp;
	//	cout << "Done" << endl;
	}
}
/*		FUNCTIONS FACILITATING FORWARD VARIABLE CALCULATION
 *
 *
 */
void HSMM::updateBtmp(colvec& _btmp, colvec& _obs)
{
	// Calculate the initial forward step
	for (uint i =0;i<this->getNumSTATES();i++)
	{
		_btmp(i) = this->getCOMPONENTS(i).getPDFValue(_obs)(0);
	}
	_btmp = _btmp/accu(_btmp);

}
void HSMM::updateBtmp(colvec& _btmp, colvec& _obs, urowvec& _ind)
{
	Gtmp->setNumVARS(_ind.n_elem);
	// Calculate the initial forward step
	for (uint i =0;i<this->getNumSTATES();i++)
	{
		// Create temporary Gaussian that uses only the specified indices:
		Gtmp->setMU(this->getMU(i)(_ind));
		Gtmp->setSIGMA(this->getSIGMA(i)(_ind,_ind));
		
		// Evaluate the gaussian probability:
		_btmp(i) = Gtmp->getPDFValue(_obs.rows(_ind))(0);
	}
	_btmp = _btmp/accu(_btmp);
}


// Equation (12): ALPHA MATRIX without observation:
void HSMM::updateALPHA(mat& _ALPHA, colvec& _S)
{
	// Help variables for vector-wise multiplications:
	Atmp1 = Pd.cols(0,PdSize-2);
	Atmp1.each_col() %= _S;
	
	Atmp2 = _ALPHA.cols(1,PdSize-1);

	_ALPHA.cols(0,PdSize-2) = Atmp2  + Atmp1;
	_ALPHA.col(PdSize-1) = _S % Pd.col(PdSize-1);
}
// Equation (12): ALPHA matrix update
void HSMM::updateALPHA(mat& _ALPHA, colvec& _S, colvec& _bmx)
{
	// Help variables for vector-wise multiplications:
	Atmp1 = Pd.cols(0,PdSize-2);
	Atmp1.each_col() %= _S;

	Atmp2 = _ALPHA.cols(1,PdSize-1);
	Atmp2.each_col() %= _bmx;

	_ALPHA.cols(0,PdSize-2) = Atmp2 + Atmp1;
	_ALPHA.col(PdSize-1) = _S % Pd.col(PdSize-1);
}


void HSMM::updateBmx(colvec& _bmx, mat& _ALPHA, colvec& _Btmp)
{
	// Equation (2) & (3): bmx update
	_bmx = _Btmp/accu(_Btmp.t()*sum(_ALPHA,1));
}	



// Equation (6): Update S
void HSMM::updateS(colvec& _S, mat& _ALPHA)
{
	// Equations (5) & (6) calculate S:
	_S = this->getTRANSITION().t()*_ALPHA.col(0);
}

// Equation (6): Update S
void HSMM::updateS(colvec& _S, mat& _ALPHA, colvec& _bmx)
{
	// Equations (5) & (6) calculate S:
	_S = this->getTRANSITION().t()*(_bmx%_ALPHA.col(0));
}



void HSMM::updateAlpha(colvec& _alpha,  mat& _ALPHA)
{
	// Calculate forward varaible
	_alpha = sum(_ALPHA,1);
	// Normalize
//	_alpha = _alpha/accu(_alpha);
}

void HSMM::updateAlpha(colvec& _alpha, mat& _ALPHA, colvec& _Btmp)
{
	// Calculate forward varaible
	_alpha = _Btmp%sum(_ALPHA,1);
	// Normalize
//	_alpha = _alpha/accu(_alpha);
}

} // End pbdlib namespace
