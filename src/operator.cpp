/*********************** OPERATOR ROS NODE **********************************
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
 * This is the source file of ros node operator based on 
 * SDK Force Dimension Version 3.4.1 using omega6/sigma7
 *
 * The purpose and use of this node is to communicate with KUKA LWR 
 * (lmtlwr ros node) and  WSG 50 Schunk gripper (wsg_command_node)
 * 
 * 
 * Authors: Burak Cizmeci 
 * e-mail: burak.cizmeci@tum.de 
 *
 * Version 22 November 2012 
 */

#include <iostream>
#include <cstdio>
#include <cstring>
#include <stdlib.h>
using namespace std;


// includes related to FD haptic device
#include "dhdc.h"
#include "drdc.h"
#include "CConstants.h"
#include "CMaths.h"
#include "CMatrix3d.h"
#include "CVector3d.h"

#define REFRESH_INTERVAL  0.1   // sec
#define _USE_MATH_DEFINES

//----ROS include----
#include "ros/ros.h"

#include "controlfunctions.h" // filters and control related functions
#include "UDPconnect.h"


/************ StartTeleoperation() function variables **************
*******************************************************************/
ros::Timer TimerHaptic;
ros::Timer TimerVideo;
ros::Timer TimerStamp;
// Settings for timers and discrete derivatives
double TimerPeriodHaptic = 0.001; // seconds 1/T Hz
double TimerPeriodVideo = 0.02; // seconds 1/T Hz
double TimerPeriodStamp = 0.001; // seconds 1/T Hz

//Thread synchronization variables are defined here

pthread_mutex_t SyncGripperForce;
//create mutex attribute variable
pthread_mutexattr_t GripForceAttr;

pthread_mutex_t SyncKUKAForce;
 //create mutex attribute variable
pthread_mutexattr_t KUKAForceAttr;


// Buraks remaining work: 
// control the gripper in real-time

//************* Omega 6 / Sigma 7 variables ********************//
ROBOTCOMMAND *KUKACommandPtr;
int FingerCount;
bool HasRot;
bool HasGrip;

//Variables for gripping
double* GripperWidthBuffer;
double* GripperSpeedBuffer;
int BufferSize = 10;
double FDminGripPos = 0.034916; // in m
double FDmaxGripPos = 0.061755; // in m
double FDgripWidth = 0.026839; // in m
double Wsg50Width = 110; // in mm (0.11 m)

/* IMPORTANT NOTE: FD accepts positions in meters and WSG-50 gripper accepts width length in millimeters and speed in millimeters/second  */	

double GrippingWidthScale = 2; // in this case the workspaces are identical and the gripper width is limited to 				FD gripper fingers
double GrippingSpeedScale = 2;
double GripSpeedPredErrVar = 0; // required for Kalman filtering of gripping speed
   
// KUKA Haptic Loop variables
double SigmaGripperForce;
double KUKAForce[3];

/********************************************************************/

/**************** UDP communication variables ***********************
********************************************************************/
#define DEFAULT_BUFFERSIZE (50000)
#define VIDLOCPORT 65540
#define VIDRMPORT 65541
#define VIDPCIP "10.152.4.41"
UDPSTRUCT* VidSockPtr; // Send video stream to the video decoder

#define TOPPCIP "10.152.4.76"
#define OPLOCPORT 9091
#define TOPRMPORT 9090
UDPSTRUCT* TopSockPtr; // Send Commands and Receive Force Feedback

#define OPVIDLOCPORT 9081
#define TOPVIDRMPORT 9080
UDPSTRUCT* TopVidSockPtr; // Receive video from the teleoperator

/*******************************************************************/

//for ROS node
ros::NodeHandle *n, *n1;
ros::Publisher* chatter_pub;
ros::Subscriber* sub;



void HapticsLoop(const ros::TimerEvent&){

// Haptic Signal Processing and control related processing
char ListenBuffer[DEFAULT_BUFFERSIZE]; // buffer holding the incoming force packets
int iReadBytes; // how many bytes read	
char SendBuffer[DEFAULT_BUFFERSIZE]; // array to packetize the command data
int SendBufferSize; // how many bytes has been written into SendBuffer



/*	
	//! Listen the force/torque packets coming from operator side
	iReadBytes = recvfrom(TopSockPtr->ListenSock, &ListenBuffer, sizeof(ListenBuffer), 0, (struct sockaddr *) &(TopSockPtr->sockLocal), &(TopSockPtr->fromlen));
	 cout << "iReadBytes =  " << iReadBytes << endl;

	if (iReadBytes>0) {

	     //! Parse the command coming from the operator	
//	     ParseCommand(KUKACommandPtr,ListenBuffer, iReadBytes);
	     cout << "iReadBytes =  " << iReadBytes << endl;
 
	}
	
*/	
	//! if the control type is velocity
	if (KUKACommandPtr->ControlType == 'V') {
		
		//! Get the Velocity and gripping positions from the haptic device
		dhdGetLinearVelocity(&KUKACommandPtr->Velocity[0],&KUKACommandPtr->Velocity[1],&KUKACommandPtr->Velocity[2],dhdGetDeviceID());

	   /*
		cout << "Debug: Vx =  " << KUKACommandPtr->Velocity[0] << endl;
		cout << "Debug: Vy =  " << KUKACommandPtr->Velocity[1] << endl;
		cout << "Debug: Vz =  " << KUKACommandPtr->Velocity[2] << endl;
           */
		//! TODO Apply compression on command & control architecture code comes here
		 
		//! Packetize the command 
		PrepareCommand(KUKACommandPtr,&(SendBuffer[0]), &SendBufferSize,KUKACommandPtr->ControlType);
		
		//! Send the command to the remote side
		if (SendBufferSize >0) {
		   sendto(TopSockPtr->SendSock, &(SendBuffer[0]), SendBufferSize, 0, (const struct sockaddr *)&(TopSockPtr->sockRemote), TopSockPtr->fromlen);
			
		}
		
		
		//! Display forces to the haptic device
	  	
		// apply all forces at once
    		//dhdSetForceAndGripperForce (force.x, force.y, force.z, gripperForce);
		//dhdSetForceAndGripperForce (KUKACommandPtr->GlobalForce[0], KUKACommandPtr->GlobalForce[1], KUKACommandPtr->GlobalForce[2], 0);


	    
	} // end of control type

	
} // end of haptics loop


/************ ListenVideo Thread *****************************/
//! Listens video from the teleoperator PC
void ListenVideo(const ros::TimerEvent&){
	
	char ListenBuffer[DEFAULT_BUFFERSIZE]; // video listen buffer
	int iReadBytes; // how many bytes received
	
	//! receive from the teleoperator PC
	iReadBytes = recvfrom(TopVidSockPtr->ListenSock, &ListenBuffer, sizeof(ListenBuffer), 0, (struct sockaddr *) &(TopVidSockPtr->sockLocal), &(TopVidSockPtr->fromlen));
	
	if (iReadBytes>0) {
		
		//send video to the decoding PC
		sendto(VidSockPtr->SendSock, &(ListenBuffer[0]), iReadBytes, 0, (const struct sockaddr *)&(VidSockPtr->sockRemote), VidSockPtr->fromlen);


	}


} // end of listen video
/*************************************************************/

/***************** ListenTimeStamp thread ********************/
void ListenTimeStamp(const ros::TimerEvent&) {

	// TODO A dummy machine constantly send time stamps to all PCs to measure the signal delays

	
	// Dont forget critical sections for this part

}
/************************************************************/


/********************* StartTeleoperation() *******************/
void StartTeleoperation () {

  // Set the gravity compensation on the haptic device for safety reasons
  dhdSetStandardGravity(9.81);
  dhdSetGravityCompensation(DHD_ON,dhdGetDeviceID());

/* 22.11.2012 Burak: The below code may be needed to program the gripper
   //! initialize buffers
   GripperWidthBuffer = (double*) malloc(BufferSize*sizeof(double)); 
   GripperSpeedBuffer = (double*) malloc(BufferSize*sizeof(double)); 

	for (int i=0;i<BufferSize;i++) {
			
		GripperWidthBuffer[i] = 0;
		GripperSpeedBuffer[i] = 0;
		
	}
	
*/
        // UDP port initialization for Video stream to decode video on the decoder PC
	
	// initialize the udp communication structure
	VidSockPtr = new UDPSTRUCT;
	VidSockPtr->localport = VIDLOCPORT;
	VidSockPtr->remoteport = VIDRMPORT;
	strcpy(VidSockPtr->RemoteHost,VIDPCIP);

	if (initNetwork(VidSockPtr) == 1) {

		cout << "Video PC communication is ready! " << endl;

	} else {

		cout << "Failed to initialize Video PC communication sockets! " << endl;	

	}

	// UDP port initialization for operator-teleoperator haptic communication
	
	// initialize the udp communication structure
	TopSockPtr = new UDPSTRUCT;
	TopSockPtr->localport = OPLOCPORT;
	TopSockPtr->remoteport = TOPRMPORT;
	strcpy(TopSockPtr->RemoteHost,TOPPCIP);

	if (initNetwork(TopSockPtr) == 1) {

		cout << "Teleoperator PC haptic communication is ready! " << endl;

	} else {

		cout << "Failed to initialize Teleoperator PC communication sockets! " << endl;	

	}

        // UDP port initialization for operator-teleoperator video communication

	// initialize the udp communication structure
	TopVidSockPtr = new UDPSTRUCT;
	TopVidSockPtr->localport = OPVIDLOCPORT;
	TopVidSockPtr->remoteport = TOPVIDRMPORT;
	strcpy(TopVidSockPtr->RemoteHost,TOPPCIP);

	if (initNetwork(TopVidSockPtr) == 1) {

		cout << "Teleoperator PC video communication is ready! " << endl;

	} else {

		cout << "Failed to initialize Teleoperator PC communication sockets! " << endl;	

	}

	// initialize KUKA command data structure pointer 
	KUKACommandPtr = new ROBOTCOMMAND;
	KUKACommandPtr->ControlType = 'V'; // this sets to velocity commanding

	// TODO Session initiation function shoul be called here
	// the haptic device type, the communication delay measure and any necessary information share should be done before the session starts
	if (SessionInitiation(TopSockPtr, VidSockPtr,KUKACommandPtr)) // this function was not implemented yet
        {

		cout << "Teleoperation session is successfully initialized " << endl;
		
	} else {

		cout << "Failed to initialize Teleoperation session " << endl;		

	}
	

	//! Timer callback initializations

	// create a timer callback for haptics loop with a prespecified frequency
	TimerHaptic = n->createTimer(ros::Duration(TimerPeriodHaptic), HapticsLoop);
	// create a video listener
	TimerVideo  = n->createTimer(ros::Duration(TimerPeriodVideo), ListenVideo); 
	// create a time listener
//	TimerStamp  = n->createTimer(ros::Duration(TimerPeriodStamp), ListenTimeStamp);  

	// Start threads!

 	ros::spin();

} // end of Start teleoperation
/*************************************************************/


/************** StopTeleoperation() **************************/
void StopTeleoperation () {

	// TODO: Release the memory and close the connections and devices safely
	
	// close connection with haptic device
  	dhdClose ();
	free(GripperWidthBuffer);
	free(GripperSpeedBuffer);

}
/*************************************************************/


// haptic device initialization
int InitHaptics () {

  if (dhdOpen () >= 0) {
    printf ("%s device detected\n", dhdGetSystemName());

   
    // default config: 1 finger, no wrist, no gripper
    FingerCount = 1;
    HasRot      = false;
    HasGrip     = false;

    switch (dhdGetSystemType ()) {
    case DHD_DEVICE_OMEGA331:
    case DHD_DEVICE_OMEGA331_LEFT:
    case DHD_DEVICE_SIGMA331:
    case DHD_DEVICE_SIGMA331_LEFT:
      HasRot      = true;
      HasGrip     = true;
      FingerCount = 2;
      break;
    case DHD_DEVICE_OMEGA33:
    case DHD_DEVICE_OMEGA33_LEFT:
    case DHD_DEVICE_DELTA6:
      HasRot = true;
      break;
    }
  }

  else {
    printf ("no device detected\n");
    dhdSleep (2.0);
    exit (0);
  }

  // 24.08.2012 Burak: Calibrate the device and position it to the center of the workspace
  printf("Calibrating the device ... \n");

  // open the first available device
  if (drdOpen () < 0) {
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr ());
    return -1;
  }
	  // perform auto-initialization
  if (drdAutoInit () < 0) {
    printf ("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr ());
    return -1;
  }

  // stop regulation (and leave force enabled)
  drdStop (true);

  printf("Calibration is done! \n");

  printf ("\n");

  return 0;
}


/************************* main() *****************************/
int main (int   argc, char **argv)
{
  cout << endl;
  cout << "Force Dimension - KUKA LWR Teleoperation " << endl;
  cout << "(C) TU München 2012 Lehrstuhl für Medientechnik" << endl;
  cout << "Author: Burak Cizmeci"<< endl;
  cout << "All Rights Reserved." << endl << endl;


  // initialize haptic devices
  InitHaptics ();

  //initialize ROS
     ros::init(argc, argv, "operator");
     n = new ros::NodeHandle;

  // Start teleoperation
  StartTeleoperation ();


  // Stop teleoperation
  StopTeleoperation();
	
   cout << "Terminating KUKA LWR Teleoperation! " << endl;
   	
   exit(0);

  return 0;
}
/*************************************************************/
