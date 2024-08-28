#ifndef GENCO_NODE_H
#define GENCO_NODE_H

#include <cmath>
#include <array>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <tuple>


// 声明函数
// Eigen::Vector3f RelativeVelocity2NewFrame(const Eigen::Vector3f& positionA, const Eigen::Vector3f& velocityA, const Eigen::Vector3f& positionB, const Eigen::Vector3f& velocityB);
std::tuple<Eigen::Vector3f, Eigen::Vector3f, float, Eigen::Matrix3f> RelativeVelocity2NewFrame(const Eigen::Vector3f& positionA, const Eigen::Vector3f& velocityA, const Eigen::Vector3f& positionB, const Eigen::Vector3f& velocityB, const float& safe_therahold);

Eigen::Matrix3f computeRotationMatrix(const float& A1, const float& B1) ;
// Eigen::Matrix3f computeRotationMatrix(const Eigen::Vector3f& from, const Eigen::Vector3f& to);

// 定义一个结构体来存储轨迹点的信息
struct TrajectoryPoint {
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration;
    float time;
};

struct Data {
    double psi_g;
    double theta_g;
    double psi_min;
    double psi_max;
    double theta_min;
    double theta_max;
    float therahold_slope;
    std::vector<Eigen::Vector2f> target_xy_;
    float V;
    std::vector<Eigen::Matrix3f> R;
    // 其他需要的数据

    // 无参构造函数
    Data() = default;

        Data(int num_uavs)
    : target_xy_(num_uavs, Eigen::Vector2f::Zero()),
        R(num_uavs, Eigen::Matrix3f::Identity()) // 初始化为单位矩阵
{}
};


// 声明生成轨迹函数
std::vector<TrajectoryPoint> generateTrajectory(const Eigen::Vector3f& start, const Eigen::Vector3f& end, float total_time, const std::vector<float>& time_intervals);

// 声明打印轨迹函数
void printTrajectory(const std::vector<TrajectoryPoint>& trajectory, const std::string& drone_name);

float calculateSlope(const Eigen::Vector3f& velocity);



double objective_function(const std::vector<double> &x, std::vector<double> &grad, void *data) ;
bool in_DRi(const double psi, const double theta) ;
void constraint_function(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data) ;

std::tuple<double, double> Velocity2Angles(const Eigen::Vector3f& V) ;
Eigen::Vector2f calculatePerpendicularIntersection(float x, float y, float k) ;
Eigen::Vector3f Angles2Velocity(const double& psi, const double& theta, const double& V);


#endif // GENCO_NODE_H