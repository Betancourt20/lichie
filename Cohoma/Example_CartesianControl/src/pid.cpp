#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"
#include "Eigen/Dense"
using namespace Eigen;
using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, Matrix<double, 6, 6> Kp, Matrix<double, 6, 6> Kd, Matrix<double, 6, 6> Ki );
        ~PIDImpl();
        Matrix<double, 6, 1> calculate( Matrix<double, 6,1 > setpoint, Matrix<double, 6, 1> pv );

    private:
        double _dt;
        double _max;
        double _min;
        Matrix<double, 6, 6> _Kp;
        Matrix<double, 6, 6> _Kd;
        Matrix<double, 6, 6> _Ki;
        Matrix<double, 6, 1> _pre_error;
        Matrix<double, 6, 1> _integral;
};


PID::PID( double dt, double max, double min, Matrix<double, 6, 6> Kp, Matrix<double, 6, 6> Kd, Matrix<double, 6, 6> Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
Matrix<double, 6, 1> PID::calculate( Matrix<double, 6, 1> setpoint, Matrix<double, 6, 1> pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, Matrix<double, 6, 6> Kp, Matrix<double, 6, 6> Kd, Matrix<double, 6, 6> Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(_pre_error.setZero(6,1)),
    _integral(_integral.setZero(6,1))
{
}

Matrix<double, 6, 1> PIDImpl::calculate( Matrix<double, 6, 1> setpoint, Matrix<double, 6, 1> pv )
{
    cout << "I'm in " << endl;
    cout << "Kp (" << _Kp << ")" << endl;

    // Calculate error
    Matrix<double, 6, 1> error = setpoint - pv;
    cout << "1" << endl;
     cout << "error (" << error << ")" << endl;
    // Proportional term
    Matrix<double, 6, 1> Pout = _Kp * error;
   cout << "2" << endl;
    // Integral term
    _integral = error + error * _dt;
    Matrix<double, 6, 1> Iout = _Ki * _integral;
     cout << "3" << endl;
     cout << "_pre_error (" << _pre_error << ")" << endl;
    // Derivative term
    Matrix<double, 6, 1> derivative = (error - _pre_error)/ _dt;
    Matrix<double, 6, 1> Dout = _Kd * derivative;
    cout << "_pre_error (" << _pre_error << ")" << endl;
  cout << "4" << endl;
    // Calculate total output
    Matrix<double, 6, 1> output = Pout + Iout + Dout;
  cout << "5" << endl;
  /*
    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;
*/
    // Save error to previous error
    _pre_error = error;
  cout << "6" << endl;
    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
