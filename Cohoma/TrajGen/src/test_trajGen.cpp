// g++ test_trajGen.cpp -o ../test -I/usr/include/python3.8 -lpython3.8


#include <tuple>
#include "../include/TrajGen/matplotlibcpp.h"
#include "../include/TrajGen/Traj_gen.hpp"

namespace plt = matplotlibcpp;

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> computeQuinticTraj(TrajGenQuintic Tj,double tf, int n)
{
    std::vector<double> qd;
    std::vector<double> d_qd;
    std::vector<double> dd_qd;
    std::vector<double> time;
    Matrix<double,3,1> traj_pt;

    double step = (tf) / n;
    for (double t = 0; t < tf; t += step)
    {
        traj_pt=Tj.eval_Traj(t);
        qd.push_back(traj_pt(0,0));
        d_qd.push_back(traj_pt(1,0));
        dd_qd.push_back(traj_pt(2,0));
        time.push_back(t);
    }

    return std::make_tuple(time, qd, d_qd, dd_qd);
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>,
 std::vector<double>, std::vector<double>, std::vector<double>,
  std::vector<double>> computeTraj6D(Traj6D Tj,double tf, int n)
{
    std::vector<double> Xd;
    std::vector<double> Yd;
    std::vector<double> Zd;
    std::vector<double> d_Xd;
    std::vector<double> d_Yd;
    std::vector<double> d_Zd;
    std::vector<double> time;
    Matrix<double,3,3> traj_pt;

    double step = (tf) / n;
    for (double t = 0; t < tf; t += step)
    {
        traj_pt=Tj.eval_X(t);
        Xd.push_back(traj_pt(0,0));
        Yd.push_back(traj_pt(1,0));
        Zd.push_back(traj_pt(2,0));

        d_Xd.push_back(traj_pt(0,1));
        d_Yd.push_back(traj_pt(1,1));
        d_Zd.push_back(traj_pt(2,1));

        time.push_back(t);
    }

    return std::make_tuple(time,Xd,Yd,Zd,d_Xd,d_Yd,d_Zd);
}

void plotResults(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> qdt)
{

    std::vector<double> time = std::get<0>(qdt);
    std::vector<double> qd = std::get<1>(qdt);
    std::vector<double> d_qd = std::get<2>(qdt);
    std::vector<double> dd_qd = std::get<3>(qdt);

    plt::figure_size(1000, 1000);
    plt::named_plot("s",time, qd);
    plt::named_plot("ds",time, d_qd);
    plt::named_plot("dds",time, dd_qd);
    plt::xlabel("time");
    plt::ylabel("SI");

    plt::legend();
    plt::show();

}

void plotXd(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>,
 std::vector<double>, std::vector<double>, std::vector<double>,
  std::vector<double>> Xdt)
{

    std::vector<double> time = std::get<0>(Xdt);
    std::vector<double> Xd = std::get<1>(Xdt);
    std::vector<double> Yd = std::get<2>(Xdt);
    std::vector<double> Zd = std::get<3>(Xdt);

    std::vector<double> dXd = std::get<4>(Xdt);
    std::vector<double> dYd = std::get<5>(Xdt);
    std::vector<double> dZd = std::get<6>(Xdt);

    plt::figure_size(1000, 1000);
    plt::subplot(1,2,1);
    plt::named_plot("x",time, Xd);
    plt::named_plot("y",time, Yd);
    plt::named_plot("z",time, Zd);
    plt::xlabel("t(s)");
    plt::ylabel("m");
    plt::legend();
    plt::subplot(1,2,2);
    plt::named_plot("x",time, dXd);
    plt::named_plot("y",time, dYd);
    plt::named_plot("z",time, dZd);
    plt::xlabel("t(s)");
    plt::ylabel("m/s");
    plt::legend();
    plt::show();

}

int main()
{
    TrajGenQuintic trajQ(10,43.5);
    double tf=20;
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> qdt = computeQuinticTraj(trajQ, tf, 1000);
    plotResults(qdt);

    Vector3d deltaX(0.5,0,0);
    double vm=0.15;

    Vector3d deltaV(0.4,-0.2,0.3);

    Traj6D trajX(deltaX,vm);
    Traj6D trajV(deltaV,vm);
    std::cout << " Trajectories duration : \n"
              << "X :\t" << trajX.get_dt() << "\t length :\t" << trajX.get_length() << "\n"
              << "V :\t" << trajV.get_dt() << "\t length :\t" << trajV.get_length() << "\n";

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>,
        std::vector<double>, std::vector<double>, std::vector<double>,   std::vector<double>> Xdt= computeTraj6D( trajX,1.5*trajX.get_dt(), 1000);
    plotXd(Xdt);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>,
        std::vector<double>, std::vector<double>, std::vector<double>,   std::vector<double>> Vdt= computeTraj6D( trajV,1.5*trajV.get_dt(), 1000);
    plotXd(Vdt);

}