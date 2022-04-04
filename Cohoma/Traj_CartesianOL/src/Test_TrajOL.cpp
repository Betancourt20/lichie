// g++ Test_TrajOL.cpp -o ../../build/test_TrajOL -I../../TrajGen/include/TrajGen -I../../_HEADERS

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
#include "Traj_gen.hpp"
#elif _WIN32
#include <Windows.h>
#include "CommunicationLayer.h"
#include "CommandLayer.h"
#include <conio.h>
#include "Eigen/Dense"
#include "pid.h"
#endif

using namespace std;
using namespace Eigen;

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

int main(int argc, char* argv[])
{
    Vector3d pos_h_c(-0.2,0,0.5);
    Vector3d pos_start;
    Vector3d delta_pos;
    double vm=0.1;
    double duration;
    double t_traj;
    int nb_step;
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
        
        cout << "Getting Current Position :" << result << endl;
        (*MyGetCartesianPosition)(CurrentPosition);

        for (int i = 0; i < devicesCount; i++)
		{
			cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

			//Setting the current device as the active device.
			MySetActiveDevice(list[i]);

			cout << "Send the robot to HOME position" << endl;
			MyMoveHome();

			TrajectoryPoint pointToSend;
			pointToSend.InitStruct();

			cout << "Send the robot to HOME position" << endl;

			//We specify that this point will be an cartesian position.
			pointToSend.Position.Type = CARTESIAN_POSITION;

			//We get the actual cartesian command of the robot.
			MyGetCartesianCommand(currentCommand);
        
        	pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X;
			pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y - 0.1f;
			pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z;
			pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
			pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
			pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ;   
            
            pos_start << currentCommand.Coordinates.X,
                        currentCommand.Coordinates.Y,
                        currentCommand.Coordinates.Z;

            delta_pos=pos_h_c-pos_start;

            Traj6D trajX(delta_pos,vm);
            duration=trajX.get_dt();

            Matrix<double,3,3> traj_pt;
            cout << "*********************************" << endl;
                std::cout << " Trajectories duration : \t"<< trajX.get_dt() 
            << "\t length :\t" << trajX.get_length() << "m \n";
            
            nb_step=int(duration/0.005);
            for (int step = 0; step < (nb_step+10); step++)
			{
                t_traj=(step*0.005);
                traj_pt=trajX.eval_X(t_traj);
                pointToSend.Position.CartesianPosition.X =traj_pt(0,0)+pos_start(0,0);
                pointToSend.Position.CartesianPosition.Y =traj_pt(1,0)+pos_start(1,0);
                pointToSend.Position.CartesianPosition.Z =traj_pt(2,0)+pos_start(2,0);
				
                //We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
				MySendBasicTrajectory(pointToSend);
#ifdef __linux__ 
		 		usleep(5000);
#elif _WIN32
				Sleep(5);	
#endif
			}

			cout << "*********************************" << endl << endl << endl;
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
