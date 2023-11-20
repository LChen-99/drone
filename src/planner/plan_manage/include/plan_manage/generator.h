#ifndef __GENERATOR
#define __GENERATOR

#include <vector>
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

typedef typename Eigen::MatrixXd MatXd;
typedef typename Eigen::VectorXd VecXd;

typedef typename Eigen::Vector2d Vec2d;
struct Traj{
    std::vector<Eigen::Vector3d> pos;
    std::vector<Eigen::Vector3d> vel;
    std::vector<Eigen::Vector3d> acc;
};

class TrajGenerator{
public:
    TrajGenerator(double T = 0.02) : T_(T){
        
    }
    bool checkTraj(const Traj& traj, double x, double y, double z);
    Traj Generator(const MatrixXd& waypoints);
    MatrixXd CalcuCoef(const MatrixXd& waypoints);
    MatrixXd GetMatrixLine(int piecewise_th, int order, double t);
    vector<Vector3d> PolyFunc(double t,  std::vector<Eigen::MatrixXd>& Polynomial_coefficients);
    void RandomGenerator(double range=1.0);
    void circleGenerate(double traj_duration_, Eigen::Vector3d start_pos, double r);
    void eightGenerate(double traj_duration_, Eigen::Vector3d start_pos, double a, double b);
    Eigen::VectorXd AllocateTime(const MatrixXd &waypoint, double max_vel_=1.0, double max_accel_=1.0) const;
    vector<Vector3d> traj_pos_;
    vector<Vector3d> traj_vel_;
    vector<Vector3d> traj_acc_;
    vector<double> traj_time_;
    Eigen::Vector3d initial_pos;
    double initial_yaw;
    Vector3d last_pos;
    double T_;
};

#endif