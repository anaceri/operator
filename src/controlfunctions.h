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

#ifndef CONTROLFUNCTIONS_H
#define CONTROLFUNCTIONS_H

#include <math.h>
#include <time.h>


//! Data structure holding the variables related to robot and sensors attached to it
typedef struct {
	
	char ControlType; // 'P' for position control,'V' for velocity control, 'B' for both
	char HapticDevice; // 'O' : omega6, 'S': Sigma7
	double Velocity[3]; // 3 DoF global velocity 
	double Position[3]; // 3 DoF global position
	double Quaternion[4];
	double Torque[3]; // 3 DoF torque
	double GlobalForce[3]; // 3 DoF global force feedback
	double FingerPosGlobal[0]; // 1 DoF finger position
	double GrippingVelocity; // 1 DoF finger velocity
	double GripperForce; //  1 DoF Force feedback for gripping

	//! Please add the additional sensor data into this data structure

}ROBOTCOMMAND;

typedef struct {

	double Wave[3];
        clock_t timestamp;

}WaveTransmit;

void SlaveWaveTransforms(double* BackwardWave,double* SlaveVelocity,double ForwardWave,double Velocity,double Force,double b); // Wave transform for slave side
void MasterWaveTransforms(double* ForwardWave,double* MasterForce,double Velocity,double Force,double BackwardWave,double b); // Wave transform for master side
void PositionToVelocity();
void VelocityToPosition();

double KalmanFiltering(double* PrevEst,double* CurrSeqSample,double* PredErrVar); // noise free velocity output
double FirstOrderDerivative(double Sample1,double Sample2,double Interval); // first order approximation for the first derivative for velocity
double SecondOrderDerivative(double Sample1,double Sample2,double Sample3,double Interval); // second order approximation for the first derivative for velocity
void Shift2DArray(double** A,int x,int y);
void Shift1DArray(double* A,int x);

// just pass the samples, coefficients and the order of the filter
// calculate the filter coefficients using matlab
double FIR_filter(double* Samples, double* Coeff, int size);

void LowPassFilter();


#endif
