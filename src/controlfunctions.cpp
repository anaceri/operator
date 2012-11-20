/*********************** CONTROL FUNCTIONS *********************************
 * Software License Agreement (BSD License)                                *
 *                                                                         *
 *  Copyright (c) 2012, Burak Cizmeci                                      *
 *  Lehrstuhl für Medientechnik                                            *
 *  Technische Universität München, Germany                                *
 *  All rights reserved.                                                   *
 *                                                                         *
 *  Redistribution and use in source and binary forms, with or without     *
 *  modification, are permitted provided that the following conditions     *
 *  are met:                                                               *
 *                                                                         *
 *  - Redistributions of source code must retain the above copyright       *
 *     notice, this list of conditions and the following disclaimer.       *
 *  - Redistributions in binary form must reproduce the above              *
 *     copyright notice, this list of conditions and the following         *
 *     disclaimer in the documentation and/or other materials provided     *
 *     with the distribution.                                              *
 *  - Neither the name of Technische Universität München nor the names of  *
 *     its contributors may be used to endorse or promote products derived *
 *     from this software without specific prior written permission.       *
 *                                                                         *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    *
 *  'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS      *
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE         *
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,    *
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,   *
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;       *
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER       *
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT     *
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN      *
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE        *
 *  POSSIBILITY OF SUCH DAMAGE.                                            *
 ***************************************************************************
 * 
 * This is the source file of Control related functions
 * 
 *
 *  
 * 
 * 
 * 
 * Authors: Burak Cizmeci 
 * e-mail: burak.cizmeci@tum.de 
 *
 * Version 20 November 2012 
 */

#include "controlfunctions.h"


void MasterWaveTransforms(double* ForwardWave,double* MasterForce,double Velocity,double Force,double BackwardWave,double b) {

	//// Wave transform with impedance matching
	//*ForwardWave =  (Velocity - (1/b)*Force)*sqrt(2*b) - BackwardWave;
	//*MasterForce = (Velocity - (1/b)*Force)*b - sqrt(2*b)*BackwardWave;

	// Wave transform without impedance matching
	*ForwardWave = sqrt(2*b)*Velocity - BackwardWave;
	*MasterForce = b*Velocity - sqrt(2*b)*BackwardWave;

}

void SlaveWaveTransforms(double* BackwardWave,double* SlaveVelocity,double ForwardWave,double Velocity,double Force,double b) {

	//// Wave transform with impedance matching
	//*BackwardWave = ForwardWave - (2/sqrt(2*b))*(Velocity*b - Force);
	//*SlaveVelocity = (2/sqrt(2*b))*ForwardWave - (1/b)*(Velocity*b - Force);

	// Wave transform without impedance matching
	*BackwardWave = ForwardWave - (2/sqrt(2*b))*Force;
	*SlaveVelocity = (sqrt(2*b)/b)*ForwardWave - (1/b)*Force;

}

double KalmanFiltering(double* PrevEst,double* CurrSeqSample,double* PredErrVar) {
	
	double CurrEst;
	double Innovation; // I
	double InnovationVar; // S
	double NoiseVar = 100; // R
	double Gain; // K
	double ProcNoiseVar = 1; // Q

	Innovation = *CurrSeqSample - *PrevEst;
	InnovationVar = *PredErrVar + NoiseVar;
	Gain = *PredErrVar/InnovationVar;

	CurrEst = *PrevEst + Gain*Innovation;

	// update values for next iteration
	*PredErrVar = *PredErrVar + ProcNoiseVar - Gain*(*PredErrVar);
	*PrevEst = CurrEst;

	return CurrEst;

}

//! 15.07.2012 Burak: This is the first order approximation to the first order derivation
double FirstOrderDerivative(double Sample1,double Sample2,double Interval) {

	return (Sample1-Sample2)/Interval;

}

//! 15.07.2012 Burak: This not the second order derivative, it is the second order approximation to the first order derivation
double SecondOrderDerivative(double Sample1,double Sample2,double Sample3,double Interval) {

	return ( 3*Sample1 - 4*Sample2 + Sample3 )/(2*Interval);

}

void Shift2DArray(double** A,int x,int y) {

	for(int i=x-2;i>=0;i--) {
	
		for (int j=0;j<y;j++) {
		
			A[i+1][j] = A[i][j];

		}
		
	
	}

}

void Shift1DArray(double* A,int x) {

		for (int j=x-2;j>=0;j--) {
		
			A[j+1] = A[j];

		}
		

}


// just pass the samples, coefficients and the order of the filter
// calculate the filter coefficients using matlab
double FIR_filter(double* Samples, double* Coeff, int size) {
    
	double Result = 0;
	for(int i= 0;i<size;i++) {
	 
		Result  = Result + Samples[i]*Coeff[i];
	
	}

	return Result;

}

