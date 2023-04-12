#include <Eigen/Dense>

// y 测量的残差force有很大噪声  网络计算的残差   
// 矩阵P   矩阵R    矩阵Q  K
// s-> 位置和速度误差  
using namespace Eigen;
class KalmanAdaptive{

public:

    KalmanAdaptive(double t) : dt(t), lamda(10){
        R = Matrix<double, 3, 3>::Identity();
        Q = Matrix<double, 9, 9>::Identity();
        P = Matrix<double, 9, 9>::Identity();
        a = Matrix<double, 9, 1>::Zero();
    }

    void update(Vector3d y_F, Vector3d s, Vector3d fai_input){
        // 更新 a_x
        Matrix<double, 3, 9> fai = Matrix<double, 3, 9>::Zero();
        fai.block(0, 0, 1, 3) = fai_input.transpose();
        fai.block(1, 3, 1, 3) = fai_input.transpose();
        fai.block(2, 6, 1, 3) = fai_input.transpose();

        Matrix<double, 9, 1>  a_minus = (1 - lamda * dt) * a;
        Matrix<double, 9, 9> P_minus = (1 - lamda)*(1 - lamda) * P + Q * dt;
        Matrix<double, 9, 3> K_temp = P_minus * fai.transpose() * (fai * P_minus * fai.transpose() + R * dt).inverse(); 
        Matrix<double, 9, 1> a_plus = a_minus - K_temp * (fai * a_minus - y_F) - P_minus * fai.transpose() * s;
        P = (Matrix<double, 9, 9>::Identity() - K_temp * fai)*  P_minus * (Matrix<double, 9, 9>::Identity() - K_temp * fai).transpose() + dt * K_temp * R * K_temp.transpose();
        a = a_plus;
        K = K_temp;
    }
    Matrix<double, 9, 1> get_a(){
        return a;
    }

private:
    Matrix<double, 3, 3> R;
    Matrix<double, 9, 9> P;
    Matrix<double, 9, 9> Q;
    Matrix<double, 9, 3> K;
    double lamda;
    double dt;
    Matrix<double, 9, 1> a;
};