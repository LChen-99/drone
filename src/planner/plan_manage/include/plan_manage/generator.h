#ifndef __GENERATOR
#define __GENERATOR

#include <vector>
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

class TrajGenerator{
public:
    TrajGenerator(double T = 0.01) : T_(T){}
    void Generator(MatrixXd waypoints);
    MatrixXd CalcuCoef(MatrixXd waypoints);
    MatrixXd GetMatrixLine(int piecewise_th, int order, double t);
    vector<Vector3d> PolyFunc(double t);

    vector<Vector3d> traj_pos_;
    vector<Vector3d> traj_vel_;
    vector<Vector3d> traj_acc_;
    vector<MatrixXd> Polynomial_coefficients_;
    vector<double> traj_time_;
    double T_;
};

#endif