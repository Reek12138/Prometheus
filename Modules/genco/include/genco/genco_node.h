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
std::tuple<Eigen::Vector3f, Eigen::Vector3f, float> RelativeVelocity2NewFrame(const Eigen::Vector3f& positionA, const Eigen::Vector3f& velocityA, const Eigen::Vector3f& positionB, const Eigen::Vector3f& velocityB, const float& safe_therahold);

Eigen::Matrix3f computeRotationMatrix(const Eigen::Vector3f& from, const Eigen::Vector3f& to);

// 定义一个结构体来存储轨迹点的信息
struct TrajectoryPoint {
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration;
    float time;
};

// 声明生成轨迹函数
std::vector<TrajectoryPoint> generateTrajectory(const Eigen::Vector3f& start, const Eigen::Vector3f& end, float total_time, const std::vector<float>& time_intervals);

// 声明打印轨迹函数
void printTrajectory(const std::vector<TrajectoryPoint>& trajectory, const std::string& drone_name);

float calculateSlope(const Eigen::Vector3f& velocity);
#endif // GENCO_NODE_H