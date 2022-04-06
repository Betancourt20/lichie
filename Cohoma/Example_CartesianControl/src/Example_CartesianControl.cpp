#ifndef UNICODE
#define UNICODE
#endif
#include "KinovaTypes.h"
#include <iostream>
#ifdef __linux__
#include <dlfcn.h>
#include <vector>
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include <stdio.h>
#include <unistd.h>
#include "Eigen/Dense"
#include "pid.h"
#elif _WIN32
#include <Windows.h>
#include "CommunicationLayer.h"
#include "CommandLayer.h"
#include <conio.h>
#include "Eigen/Dense"
#include "pid.h"
#endif

using namespace std;
using Eigen::MatrixXf;
using Eigen::VectorXf;


//A handle to the API._
void * commandLayer_handle;


//Function pointers to the functions we need
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MySendBasicTrajectory)(TrajectoryPoint trajectory);
int(*MyGetCartesianPosition)(CartesianPosition &);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);
int(*MyMoveHome)();
int(*MyInitFingers)();
int(*MyGetCartesianCommand)(CartesianPosition &);


void GoToPoint(TrajectoryPoint pointToSend, Matrix< double, 6, 1 > PosDes)
{
    pointToSend.Position.Type = CARTESIAN_POSITION;

    pointToSend.Position.CartesianPosition.X = PosDes(0);
    pointToSend.Position.CartesianPosition.Y = PosDes(1);
    pointToSend.Position.CartesianPosition.Z = PosDes(2);
    pointToSend.Position.CartesianPosition.ThetaX = PosDes(3);
    pointToSend.Position.CartesianPosition.ThetaY = PosDes(4);
    pointToSend.Position.CartesianPosition.ThetaZ = PosDes(5);

    pointToSend.Position.Fingers.Finger1 = 0;
    pointToSend.Position.Fingers.Finger2 = 0;
    pointToSend.Position.Fingers.Finger3 = 0;
    //add a velocity limitation
    pointToSend.Limitations.speedParameter1 = 0.10; // translation limited to 0.10 m/sec
    pointToSend.Limitations.speedParameter2 = 0.5; //rotation limited to 05. rad/sec
    pointToSend.Limitations.speedParameter3 = 0; //speed parameter for the fingers. Not considered in the trajectory functions
    //enable limitations
    pointToSend.LimitationsActive = 1;
    pointToSend.Position.HandMode = HAND_NOMOVEMENT; // no movement from the fingers

    (*MySendBasicTrajectory)(pointToSend);
}

int main(int argc, char* argv[])
{
    MatrixXd pos(6,1);
    MatrixXd pos_des(6,1);
	int programResult = 0;
  	CartesianPosition CurrentPosition;

	//We load the API
	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	//We load the functions from the library
	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	MyMoveHome = (int (*)()) dlsym(commandLayer_handle,"MoveHome");
	MyInitFingers = (int (*)()) dlsym(commandLayer_handle,"InitFingers");
	MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
	MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
	MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
	MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianPosition");

	//Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) || (MyGetCartesianPosition == NULL) ||
		(MyGetDevices == NULL) || (MySetActiveDevice == NULL)  ||(MyMoveHome == NULL) ||  (MyInitFingers == NULL))

	{
		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
		programResult = 0;
	}
	else
	{
	cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;
	int result = (*MyInitAPI)();
	cout << "Initialization's result :" << result << endl;

	KinovaDevice list[MAX_KINOVA_DEVICE];
	int devicesCount = MyGetDevices(list, result);


	for (int i = 0; i < devicesCount; i++)
	{
		cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);
		cout << "Send the robot to HOME position" << endl;
        MyMoveHome();
		cout << "Initializing the fingers" << endl;
		MyInitFingers();
		TrajectoryPoint pointToSend;
		pointToSend.InitStruct();

		//	 Pos home standard
	/*	pos_des <<  0.21146,
              -0.265456,
              0.504568,
              1.66124,
              1.108,
              0.120692;*/



       pos_des <<  -0.183982,  // Manipulator up position
                    0.00861586,
                    0.156995,
                    0.831117,
                    1.3696,
                    0.735864;

      for (int i = 0; i < 200; i++)
      {
        GoToPoint(pointToSend, pos_des);
        usleep(5000);

      }
      /*
        pos_des << 0.0418865, // Ready to push position
                   0.634523,
                   0.224163,
                   -0.329754,
                   1.36459,
                   2.88776;*/


	}

	cout << endl << "C L O S I N G   A P I" << endl;
	result = (*MyCloseAPI)();
	programResult = 1;
	}

	dlclose(commandLayer_handle);
	return programResult;

}
