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
int(*MySendBasicTrajectory)(TrajectoryPoint command);
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
    		AngularPosition currentCommand;

        //We specify that this point will be an angular(joint by joint) position.
			pointToSend.Position.Type = ANGULAR_POSITION;

    //We get the actual angular command of the robot.
    MyGetAngularCommand(currentCommand);

    pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1 + 180;
    pointToSend.Position.Actuators.Actuator2 = currentCommand.Actuators.Actuator2 + 56;
    pointToSend.Position.Actuators.Actuator3 = currentCommand.Actuators.Actuator3 + 27 ;
    pointToSend.Position.Actuators.Actuator4 = currentCommand.Actuators.Actuator4 + 266;
    pointToSend.Position.Actuators.Actuator5 = currentCommand.Actuators.Actuator5 + 176;
    pointToSend.Position.Actuators.Actuator6 = currentCommand.Actuators.Actuator6 + 288;
    
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
	MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");
   	MyGetCartesianPosition = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianPosition");
    MyGetAngularCommand = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularCommand");
#endif

	//Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) || (MyGetCartesianPosition == NULL) || 
		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetCartesianCommand == NULL) ||
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
              
       double kpx = 2, kpy=2, kpz=2, kpwx=0, kpwy=0, kpwz=0;
       
        Kp << kpx,0,0,0,0,0,
              0,kpy,0,0,0,0,
              0,0,kpz,0,0,0,
              0,0,0,kpwx,0,0,
              0,0,0,0,kpwy,0,
              0,0,0,0,0,kpwz;
        
        double kdx = 0.05, kdy=0.05, kdz=0.05, kdwx=0, kdwy=0, kdwz=0;
        Kd << kdx,0,0,0,0,0,
              0,kdy,0,0,0,0,
              0,0,kdz,0,0,0,
              0,0,0,kdwx,0,0,
              0,0,0,0,kdwy,0,
              0,0,0,0,0,kdwy;
              
        double kix = 0.0, kiy=0, kiz=0, kiwx=0, kiwy=0, kiwz=0;      
        Ki << kix,0,0,0,0,0,
              0,kiy,0,0,0,0,
              0,0,kiz,0,0,0,
              0,0,0,kiwx,0,0,
              0,0,0,0,kiwy,0,
              0,0,0,0,0,kiwz;     
                      
          //PID2 pid2 = PID2(0.1, 100, -100, 0.1, 0.01, 0.5);
         PID pid = PID(0.1, 100, -100, Kp, Kd, Ki);
        
		for (int i = 0; i < devicesCount; i++)
		{
			cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

			//Setting the current device as the active device.
			MySetActiveDevice(list[i]);
			

			cout << "Send the robot to HOME position" << endl;
            HomePosition();
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
            pos_des <<  0.21146,
                        -0.265456, 
                        0.804568, 
                        1.66124,
                        1.108,
                        0.120692;
           
			
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
			//cout << "Pos" << pos << endl;
			//cout << "Pos_des" << xd_des(0) << endl;	
			
			pointToSend.Position.Type = CARTESIAN_VELOCITY;

			pointToSend.Position.CartesianPosition.X = xd_des(0);
			pointToSend.Position.CartesianPosition.Y = xd_des(1); //Move along Y axis at 20 cm per second
			pointToSend.Position.CartesianPosition.Z = xd_des(2);
			pointToSend.Position.CartesianPosition.ThetaX = 0;
			pointToSend.Position.CartesianPosition.ThetaY = 0;
			pointToSend.Position.CartesianPosition.ThetaZ = 0;

			pointToSend.Position.Fingers.Finger1 = 0;
			pointToSend.Position.Fingers.Finger2 = 0;
			pointToSend.Position.Fingers.Finger3 = 0;	
       	       
			//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
				MySendBasicTrajectory(pointToSend);
#ifdef __linux__ 
		 		usleep(5000);
#elif _WIN32
				Sleep(5);	
#endif
			}	
				
							/*
			//We specify that this point will be used an angular(joint by joint) velocity vector.
			pointToSend.Position.Type = CARTESIAN_VELOCITY;

			pointToSend.Position.CartesianPosition.X = 0;
			pointToSend.Position.CartesianPosition.Y = -0.15; //Move along Y axis at 20 cm per second
			pointToSend.Position.CartesianPosition.Z = 0;
			pointToSend.Position.CartesianPosition.ThetaX = 0;
			pointToSend.Position.CartesianPosition.ThetaY = 0;
			pointToSend.Position.CartesianPosition.ThetaZ = 0;

			pointToSend.Position.Fingers.Finger1 = 0;
			pointToSend.Position.Fingers.Finger2 = 0;
			pointToSend.Position.Fingers.Finger3 = 0;

			for (int i = 0; i < 200; i++)
			{
				//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
				MySendBasicTrajectory(pointToSend);
#ifdef __linux__ 
		 		usleep(5000);
#elif _WIN32
				Sleep(5);	
#endif
			}
			
			cout << "Send the robot to HOME position" << endl;
			MyMoveHome();*/
/*
			pointToSend.Position.CartesianPosition.Y = 0;
			pointToSend.Position.CartesianPosition.Z = 0.1;

			for (int i = 0; i < 200; i++)
			{
				//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
				MySendBasicTrajectory(pointToSend);
#ifdef __linux__ 
		 		usleep(5000);
#elif _WIN32
				Sleep(5);	
#endif
			}

			cout << "Send the robot to HOME position" << endl;
			MyMoveHome();

			//We specify that this point will be an angular(joint by joint) position.
			pointToSend.Position.Type = CARTESIAN_POSITION;

			//We get the actual angular command of the robot.
			MyGetCartesianCommand(currentCommand);

			pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X;
			pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y - 0.1f;
			pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z;
			pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
			pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
			pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ;

			cout << "*********************************" << endl;
			cout << "Sending the first point to the robot." << endl;
			MySendBasicTrajectory(pointToSend);

			pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z + 0.1f;
			cout << "Sending the second point to the robot." << endl;
			MySendBasicTrajectory(pointToSend);

			cout << "*********************************" << endl << endl << endl;
			*/
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
