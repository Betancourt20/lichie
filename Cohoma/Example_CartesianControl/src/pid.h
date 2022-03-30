#ifndef _PID_H_
#define _PID_H_
#include "Eigen/Dense"
using namespace Eigen;


class PIDImpl;
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double dt, double max, double min, Matrix<double, 6, 6> Kp, Matrix<double, 6, 6> Kd, Matrix<double, 6, 6> Ki );

        // Returns the manipulated variable given a setpoint and current process value
        Matrix<double, 6, 1>  calculate( Matrix<double, 6, 1> setpoint, Matrix<double, 6, 1> pv );
        ~PID();
             

    private:
        PIDImpl *pimpl;
        

        
};;

#endif
