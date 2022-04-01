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

void plotResults(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> qdt)
{

    std::vector<double> time = std::get<0>(qdt);
    std::vector<double> qd = std::get<1>(qdt);
    std::vector<double> d_qd = std::get<2>(qdt);
    std::vector<double> dd_qd = std::get<3>(qdt);

    plt::figure_size(1000, 1000);
    plt::plot(time, qd);
    plt::plot(time, d_qd);
    plt::plot(time, dd_qd);
    plt::xlabel("time");
    plt::ylabel("pos");

    plt::show();
}

int main()
{
    TrajGenQuintic trajQ(10,43.5);
    double tf=20;
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> qdt = computeQuinticTraj(trajQ, tf, 1000);
    plotResults(qdt);
}