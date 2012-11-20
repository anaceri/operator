/*********************** UDP COMMUNICATION *********************************
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
 * This is the source file of UDP communication
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




#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <unistd.h>
#include <fcntl.h>

#include "controlfunctions.h"



typedef struct {

	int SendSock;
	int ListenSock;
	struct sockaddr_in sockLocal;
	struct sockaddr_in sockRemote;
	int localport;
	int remoteport;
	socklen_t fromlen;
	char RemoteHost[20];

}UDPSTRUCT;

int initNetwork(UDPSTRUCT* SocketPtr);
void ParseCommand(ROBOTCOMMAND *RobotPtr,char* DataBuffer, int BufferSize);
void PrepareCommand(ROBOTCOMMAND *RobotPtr,char* DataBuffer, int* BufferSize,char CommandType);
int SessionInitiation(UDPSTRUCT *Opsockptr, UDPSTRUCT *Vidsockptr,ROBOTCOMMAND *Kukaptr);

int initNetwork(UDPSTRUCT* SocketPtr){


	//! UDP packet sender set-up
        

		
        if((SocketPtr->SendSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {

		printf("Can not create sender socket!\n");
 		return -1;
	}
	

	//! address and port settings for remote server
	memset(&(SocketPtr->sockRemote),0, sizeof(struct sockaddr_in));
	SocketPtr->sockRemote.sin_family = AF_INET;
        SocketPtr->sockRemote.sin_addr.s_addr = inet_addr(SocketPtr->RemoteHost);
     	SocketPtr->sockRemote.sin_port = htons(SocketPtr->remoteport);
		
	//! UDP packet receiver set-up
	 if((SocketPtr->ListenSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {

		printf("Can not create receiver socket!\n");
 		return -1;
	}
	
	SocketPtr->sockLocal.sin_family = AF_INET;
	SocketPtr->sockLocal.sin_port = htons(SocketPtr->localport);
	SocketPtr->sockLocal.sin_addr.s_addr = INADDR_ANY;
	memset(&(SocketPtr->sockLocal.sin_zero), '\0', 8);
	


	if(bind(SocketPtr->ListenSock, (struct sockaddr *)&(SocketPtr->sockLocal), sizeof(SocketPtr->sockLocal)) < 0) {

               printf("Can not bind in server!\n");
	       return -1;
	}
	
	fcntl(SocketPtr->SendSock, F_SETFL, O_NONBLOCK); // set to non-blocking mode

	SocketPtr->fromlen = sizeof(struct sockaddr_in);
	
	
	return 1;

}

void ParseCommand(ROBOTCOMMAND *RobotPtr,char* DataBuffer, int BufferSize) {

	char CommandType;
	
	memcpy(&(CommandType),&(DataBuffer[0]),sizeof(char));
	

	switch(CommandType) {
		
		// Velocity command packet
		case 'V':
			
			memcpy(&(RobotPtr->Velocity[0]),&(DataBuffer[sizeof(char)]),3*sizeof(double));

		break;

		// Quaternion command packet
		case 'Q':
			
			memcpy(&(RobotPtr->Quaternion[0]),&(DataBuffer[sizeof(char)]),4*sizeof(double));

		break;
		
		// Poisition command packet
		case 'P':

			memcpy(&(RobotPtr->Position[0]),&(DataBuffer[sizeof(char)]),3*sizeof(double));

		break;

		// Force Feedback packet
		case 'F':

			memcpy(&(RobotPtr->GlobalForce[0]),&(DataBuffer[sizeof(char)]),3*sizeof(double));

		break;

		// Torque packet
		case 'T':
			
			memcpy(&(RobotPtr->Torque[0]),&(DataBuffer[sizeof(char)]),3*sizeof(double));

		break;

		// All in one packet: writes the entire structure into one packet
		case 'A':

			memcpy(&(RobotPtr->Velocity[0]),&(DataBuffer[sizeof(char)]),3*sizeof(double));
			memcpy(&(RobotPtr->Position[0]),&(DataBuffer[sizeof(char)+3*sizeof(double)]),3*sizeof(double));
			memcpy(&(RobotPtr->Quaternion[0]),&(DataBuffer[sizeof(char)+6*sizeof(double)]),4*sizeof(double));
			memcpy(&(RobotPtr->Torque[0]),&(DataBuffer[sizeof(char)+10*sizeof(double)]),3*sizeof(double));
			memcpy(&(RobotPtr->GlobalForce[0]),&(DataBuffer[sizeof(char)+13*sizeof(double)]),3*sizeof(double));
			
			
		break;

		default:
			printf("Wrong Command Type is chosen! \n");
		break;


            }




}

void PrepareCommand(ROBOTCOMMAND *RobotPtr,char* DataBuffer, int* BufferSize,char CommandType) {


	switch(CommandType) {
		
		// Velocity command packet
		case 'V':
			*BufferSize = 3*sizeof(double)+sizeof(char);
			DataBuffer = (char*)malloc(sizeof(char)*(*BufferSize));
			memcpy(&(DataBuffer[0]),&(CommandType),sizeof(char));
			memcpy(&(DataBuffer[sizeof(char)]),&(RobotPtr->Velocity[0]),3*sizeof(double));
		break;

		// Quaternion command packet
		case 'Q':
			*BufferSize = 4*sizeof(double)+sizeof(char);
			DataBuffer = (char*)malloc(sizeof(char)*(*BufferSize));
			memcpy(&(DataBuffer[0]),&(CommandType),sizeof(char));
			memcpy(&(DataBuffer[sizeof(char)]),&(RobotPtr->Quaternion[0]),4*sizeof(double));
		break;
		
		// Poisition command packet
		case 'P':
			*BufferSize = 3*sizeof(double)+sizeof(char);
			DataBuffer = (char*)malloc(sizeof(char)*(*BufferSize));
			memcpy(&(DataBuffer[0]),&(CommandType),sizeof(char));
			memcpy(&(DataBuffer[sizeof(char)]),&(RobotPtr->Position[0]),3*sizeof(double));
		break;

		// Force Feedback packet
		case 'F':
			*BufferSize = 3*sizeof(double)+sizeof(char);
			DataBuffer = (char*)malloc(sizeof(char)*(*BufferSize));
			memcpy(&(DataBuffer[0]),&(CommandType),sizeof(char));
			memcpy(&(DataBuffer[sizeof(char)]),&(RobotPtr->GlobalForce[0]),3*sizeof(double));
		break;

		// Torque packet
		case 'T':
			*BufferSize = 3*sizeof(double)+sizeof(char);
			DataBuffer = (char*)malloc(sizeof(char)*(*BufferSize));
			memcpy(&(DataBuffer[0]),&(CommandType),sizeof(char));
			memcpy(&(DataBuffer[sizeof(char)]),&(RobotPtr->Torque[0]),3*sizeof(double));
		break;

		// All in one packet: writes the entire structure into one packet
		case 'A':

			*BufferSize = 16*sizeof(double)+sizeof(char); // Gripper related data is missing here!
			DataBuffer = (char*)malloc(sizeof(char)*(*BufferSize));
			memcpy(&(DataBuffer[0]),&(CommandType),sizeof(char));
			memcpy(&(DataBuffer[sizeof(char)]),&(RobotPtr->Velocity[0]),3*sizeof(double));
			memcpy(&(DataBuffer[sizeof(char)+3*sizeof(double)]),&(RobotPtr->Position[0]),3*sizeof(double));
			memcpy(&(DataBuffer[sizeof(char)+6*sizeof(double)]),&(RobotPtr->Quaternion[0]),4*sizeof(double));
			memcpy(&(DataBuffer[sizeof(char)+10*sizeof(double)]),&(RobotPtr->Torque[0]),3*sizeof(double));
			memcpy(&(DataBuffer[sizeof(char)+13*sizeof(double)]),&(RobotPtr->GlobalForce[0]),3*sizeof(double));
			
			
		break;

		default:
			printf("Wrong Command Type is chosen! \n");
		break;

	
	
	}


}

int SessionInitiation(UDPSTRUCT *Opsockptr, UDPSTRUCT *Vidsockptr,ROBOTCOMMAND *Kukaptr) {


	return 1;
}





