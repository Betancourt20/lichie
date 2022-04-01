#ifndef _TRAJ_GEN_HPP_
#define _TRAJ_GEN_HPP_

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

class TrajGenQuintic
{
    public:
        TrajGenQuintic();
        TrajGenQuintic(const TrajGenQuintic& in):
        _coef(in.coef()),
        _tt(in.tt()),
        _x(in.x())
        {}
        TrajGenQuintic(const double& tt,
            const double& x);
        virtual ~TrajGenQuintic();

        Matrix<double,3,1>& coef() const {return (Matrix<double,3,1>&)(this->_coef);}
        double& tt() const {return (double &)(this->_tt);}
        double& x() const {return (double &)(this->_x);}

        bool set_tt(const double& tt);
        bool set_x(const double& x);

        Matrix<double,3,1> eval_Traj(const double& t);

    protected:
        Matrix<double,3,1> _coef;
        double _tt;
        double _x;
        bool eval_coef();

};

TrajGenQuintic::TrajGenQuintic():
    _coef(Matrix<double,3,1>::Zero()),
    _tt(1),
    _x(1)
    {}
TrajGenQuintic::TrajGenQuintic(const double& tt,const double& x)
{
    if(this->set_tt(tt) && this->set_x(x)) this->eval_coef();
}

TrajGenQuintic::~TrajGenQuintic()
{}

bool TrajGenQuintic::set_tt(const double& tt)
{
    _tt=tt;
    return true;
}

bool TrajGenQuintic::set_x(const double& x)
{
    _x=x;
    return true;
}

bool TrajGenQuintic::eval_coef()
{
    Matrix<double,3,3> X;
    Matrix<double,3,1> B;
    X(0, 0) = std::pow(_tt, 3);
    X(0, 1) = std::pow(_tt, 4);
    X(0, 2) = std::pow(_tt, 5);

    X(1, 0) = 3 * std::pow(_tt, 2);
    X(1, 1) = 4 * std::pow(_tt, 3);
    X(1, 2) = 5 * std::pow(_tt, 4);

    X(2, 0) = 6 * _tt;
    X(2, 1) = 12 * std::pow(_tt, 2);
    X(2, 2) = 20 * std::pow(_tt, 3);

    B(0, 0) = _x;
    B(1, 0) = 0;
    B(2, 0) = 0;

    _coef=(X.inverse() * B);

    return true;
}


Matrix<double,3,1> TrajGenQuintic::eval_Traj(const double& t)
{
    Matrix<double,3,1> traj_pt;
    
    if (t<0)
    {
        traj_pt.setZero();
    }
    if (t>_tt)
    {
        traj_pt.setZero();
        traj_pt(0,0)=_x;   
    }
    else
        {
        traj_pt(0,0) = _coef(0) * std::pow(t, 3) + _coef(1) * std::pow(t, 4) + _coef(2) * std::pow(t, 5);
        traj_pt(1,0) =  3 * _coef(0) * std::pow(t, 2) + 4 * _coef(1) * std::pow(t, 3) + 5 * _coef(2) * std::pow(t, 4);
        traj_pt(2,0)=   6 * _coef(0) * t + 12 * _coef(1) * std::pow(t, 2) + 20 * _coef(2) * std::pow(t, 3);
    }
    return traj_pt;
}

        
        


#endif