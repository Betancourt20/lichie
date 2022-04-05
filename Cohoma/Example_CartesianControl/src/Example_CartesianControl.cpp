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


//A handle to the API.
#ifdef __linux__
void * commandLayer_handle;
#elif _WIN32
HINSTANCE commandLayer_handle;
#endif

//Function pointers to the functions we need
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MySendBasicTrajectory)(TrajectoryPoint trajectory);
int(*MySendAdvanceTrajectory)(TrajectoryPoint trajectory);
int(*MyGetCartesianPosition)(CartesianPosition &);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);
int(*MyMoveHome)();
int(*MyInitFingers)();
int(*MyGetCartesianCommand)(CartesianPosition &);
int(*MyGetAngularCommand)(AngularPosition &);



void HomePosition(){

  cout << "Going to Home Position";
        TrajectoryPoint pointToSend;
        pointToSend.InitStruct();
    		//AngularPosition currentCommand;

        //We specify that this point will be an angular(joint by joint) position.
			pointToSend.Position.Type = ANGULAR_POSITION;

    //We get the actual angular command of the robot.
    //MyGetAngularCommand(currentCommand);

    pointToSend.Position.Actuators.Actuator1 =  5;
    pointToSend.Position.Actuators.Actuator2 =  234;
    pointToSend.Position.Actuators.Actuator3 =  80 ;
    pointToSend.Position.Actuators.Actuator4 =  359;
    pointToSend.Position.Actuators.Actuator5 =  290;
    pointToSend.Position.Actuators.Actuator6 =  359;

    cout << "*********************************" << endl;
	cout << "Sending the first point to the robot." << endl;
	MySendBasicTrajectory(pointToSend);
}

int main(int argc, char* argv[])
{
    MatrixXd Kp(6,6);
    MatrixXd Kd(6,6);
    MatrixXd Ki(6,6);
    MatrixXd pos(6,1);
    MatrixXd pos_des(6,1);
    CartesianPosition currentCommand;
	int programResult = 0;
  	CartesianPosition CurrentPosition;





#ifdef __linux__
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
  MySendAdvanceTrajectory = (int (*)(TrajectoryPoint trajectory)) dlsym(commandLayer_handle,"SendAdvanceTrajectory");
	MyGetCartesianCommand = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianCommand");
	MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianPosition");
  MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");

#elif _WIN32
	//We load the API.
	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

	//We load the functions from the library
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
	MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");
	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
	MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
  MySendAdvanceTrajectory = (int (*)(TrajectoryPoint trajectory)) dlsym(commandLayer_handle,"SendAdvanceTrajectory");
	MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");
 	MyGetCartesianPosition = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianPosition");
  MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");


#endif

	//Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) || (MyGetCartesianPosition == NULL) ||
		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MySendAdvanceTrajectory == NULL) || (MyGetCartesianCommand == NULL) ||
		(MyMoveHome == NULL) ||  (MyInitFingers == NULL))

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

		/*	 Pos home standard
		pos_des <<  0.21146,
                        -0.265456,
                        0.504568,
                        1.66124,
                        1.108,
                        0.120692;

		*/

           pos_des <<  -0.183982,
                        0.00861586,
                        0.156995,
                        0.831117,
                        1.3696,
                        0.735864;


	       	for (int i = 0; i < 300; i++)
			{

		//cout << "Tracking desired position :" << result << endl;
       	       (*MyGetCartesianPosition)(CurrentPosition);

	      pos << CurrentPosition.Coordinates.X,
		           CurrentPosition.Coordinates.Y,
		           CurrentPosition.Coordinates.Z,
		           CurrentPosition.Coordinates.ThetaX,
		           CurrentPosition.Coordinates.ThetaY,
		           CurrentPosition.Coordinates.ThetaZ;
	//We specify that this point will be used an angular(joint by joint) velocity vector.

			MatrixXd xd_des = pid.calculate(pos_des, pos);
			pointToSend.Position.Type = CARTESIAN_POSITION;

			pointToSend.Position.CartesianPosition.X = pos_des(0);//xd_des(0);
			pointToSend.Position.CartesianPosition.Y = pos_des(1);//xd_des(1);
			pointToSend.Position.CartesianPosition.Z = pos_des(2);//xd_des(2);
			pointToSend.Position.CartesianPosition.ThetaX = pos_des(3);
			pointToSend.Position.CartesianPosition.ThetaY = pos_des(4);
			pointToSend.Position.CartesianPosition.ThetaZ = pos_des(5);

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
			//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
					(*MySendBasicTrajectory)(pointToSend);
#ifdef __linux__
		 		usleep(5000);
#elif _WIN32
				Sleep(5);
#endif
			}

      pos_des  <<  0.0418865,
                   0.634523,
                   0.224163,
                   -0.329754,
                   1.36459,
                   2.88776;


      for (int i = 0; i < 300; i++)
  {


  pointToSend.Position.Type = CARTESIAN_POSITION;

  pointToSend.Position.CartesianPosition.X = pos_des(0);//xd_des(0);
  pointToSend.Position.CartesianPosition.Y = pos_des(1);//xd_des(1);
  pointToSend.Position.CartesianPosition.Z = pos_des(2);//xd_des(2);
  pointToSend.Position.CartesianPosition.ThetaX = pos_des(3);
  pointToSend.Position.CartesianPosition.ThetaY = pos_des(4);
  pointToSend.Position.CartesianPosition.ThetaZ = pos_des(5);

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
  //We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
      (*MySendBasicTrajectory)(pointToSend);
#ifdef __linux__
    usleep(5000);
#elif _WIN32
    Sleep(5);
#endif
  }
  usleep(1000000);
  pos_des  <<  0.3,
               0.634523,
               0.224163,
               -0.329754,
               1.36459,
               2.88776;


  for (int i = 0; i < 300; i++)
{


pointToSend.Position.Type = CARTESIAN_POSITION;

pointToSend.Position.CartesianPosition.X = pos_des(0);//xd_des(0);
pointToSend.Position.CartesianPosition.Y = pos_des(1);//xd_des(1);
pointToSend.Position.CartesianPosition.Z = pos_des(2);//xd_des(2);
pointToSend.Position.CartesianPosition.ThetaX = pos_des(3);
pointToSend.Position.CartesianPosition.ThetaY = pos_des(4);
pointToSend.Position.CartesianPosition.ThetaZ = pos_des(5);

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
//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
  (*MySendBasicTrajectory)(pointToSend);
#ifdef __linux__
usleep(5000);
#elif _WIN32
Sleep(5);
#endif
}


		}

		cout << endl << "C L O S I N G   A P I" << endl;
		result = (*MyCloseAPI)();
		programResult = 1;
	}

#ifdef __linux__
	dlclose(commandLayer_handle);
#elif _WIN32
	FreeLibrary(commandLayer_handle);
#endif

	return programResult;

}
