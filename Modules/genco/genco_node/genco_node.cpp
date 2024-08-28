#include "genco_node.h"
#include <Eigen/Dense>
#include <cmath>
#include <tuple>

// 计算无人机A在以B为z轴且静止的坐标系下的相对速度,返回无人机B的坐标，无人机a的总速度，安全斜率
std::tuple<Eigen::Vector3f, Eigen::Vector3f, float, Eigen::Matrix3f> RelativeVelocity2NewFrame(const Eigen::Vector3f& positionA, const Eigen::Vector3f& velocityA, const Eigen::Vector3f& positionB, const Eigen::Vector3f& velocityB, const float& safe_therahold ) {
    // 计算相对位置和速度
    Eigen::Vector3f relativePosition = positionB - positionA;
    Eigen::Vector3f relativeVelocity = velocityA - velocityB;

    // // 定义新坐标系的 z 轴方向
    // Eigen::Vector3f zAxisNewFrame = relativePosition.normalized();

    // // 原始坐标系的 z 轴方向
    // Eigen::Vector3f zAxisOriginal(0, 0, 1);

    // 计算旋转矩阵
    float alpha_ij = std::atan2(positionB[1] - positionA[1], positionB[0] - positionA[0]);
    float beta_ij = std::atan2(positionB[2] - positionA[2], std::sqrt(std::pow(positionB[0] - positionA[0], 2) + std::pow(positionB[1]- positionA[1], 2)));
    Eigen::Matrix3f rotationMatrix = computeRotationMatrix(alpha_ij, beta_ij);

    // 旋转相对速度到新坐标系
    Eigen::Vector3f vel = rotationMatrix * relativeVelocity;
    Eigen::Vector3f pos = rotationMatrix * relativePosition;

    float c_ = relativePosition.norm();
    float b_ = std::sqrt(c_*c_ - safe_therahold*safe_therahold);
    float slope = b_/safe_therahold;

    return std::make_tuple(pos, vel, slope, rotationMatrix);
    }

Eigen::Matrix3f computeRotationMatrix(const float& alpha, const float& beta) {
    Eigen::Matrix3f R_ij ;

    R_ij <<  sin(alpha),  sin(beta) * cos(alpha),  cos(beta) * cos(alpha),
        -cos(alpha),  sin(beta) * sin(alpha),  cos(beta) * sin(alpha),
        0,           -cos(beta),              sin(beta);

    return R_ij.transpose();
}


// 生成轨迹
std::vector<TrajectoryPoint> generateTrajectory(const Eigen::Vector3f& start, const Eigen::Vector3f& end, float total_time, const std::vector<float>& time_intervals) {
    Eigen::Vector3f velocity = (end - start) / total_time;
    float V = velocity.norm();
    Eigen::Vector3f acceleration(0.0f, 0.0f, 0.0f); // 匀速运动，加速度为零
    std::vector<TrajectoryPoint> trajectory;

    for (float t : time_intervals) {
        Eigen::Vector3f position = start + velocity * t;
        trajectory.push_back({position, velocity, acceleration, t});
    }

    return trajectory;
}

// 打印轨迹
void printTrajectory(const std::vector<TrajectoryPoint>& trajectory, const std::string& drone_name) {
    std::cout << "Trajectory for " << drone_name << ":\n";
    for (const auto& point : trajectory) {
        std::cout << "Time: " << point.time
                  << " Position: (" << point.position.x() << ", " << point.position.y() << ", " << point.position.z() << ")"
                  << " Velocity: (" << point.velocity.x() << ", " << point.velocity.y() << ", " << point.velocity.z() << ")"
                  << " Acceleration: (" << point.acceleration.x() << ", " << point.acceleration.y() << ", " << point.acceleration.z() << ")\n";
    }
}

//计算速度斜率，斜率小于阈值说明在冲突范围外
float calculateSlope(const Eigen::Vector3f& velocity) {
    // 计算xy平面的速度
    float xy_speed = std::sqrt(velocity.x() * velocity.x() + velocity.y() * velocity.y());
    
    // 如果xy_speed为0，斜率将趋于无穷大，为了避免除以0的情况，可以处理为返回一个大值或异常处理
    if (xy_speed == 0) {
        return std::numeric_limits<float>::infinity(); // 或者返回一个你认为合适的值
    }
    
    // 计算斜率 z / xy_speed
    float slope = velocity.z() / xy_speed;
    
    return slope;
}

// 定义一个函数来计算偏航角(ψ)和俯仰角(θ)
std::tuple<double, double> Velocity2Angles(const Eigen::Vector3f& V) {
    double vx = V.x();
    double vy = V.y();
    double vz = V.z();

    // 计算偏航角 ψ (arctan2(vy, vx))
    double psi = std::atan2(vy, vx);
    
    // 计算俯仰角 θ (arctan2(vz, sqrt(vx^2 + vy^2)))
    double theta = std::atan2(vz, std::sqrt(vx * vx + vy * vy));
    
    // 返回计算得到的角度
    return std::make_tuple(psi, theta);
}

Eigen::Vector3f Angles2Velocity(const double& psi, const double& theta, const double& V){
    Eigen::Vector3f v;
    v[0] = V * cos(psi) * cos(theta);
    v[1] = V * sin(psi) * cos(theta);
    v[2] = V * sin(theta);

    return v;
}


// 定义一个函数来计算垂足的交点坐标，并返回 Eigen::Vector2f
Eigen::Vector2f calculatePerpendicularIntersection(float x, float y, float k) {
    // 计算垂线斜率 k_perp = -1/k
    float k_perp = -1.0f / k;
    
    // 使用直线方程 y = kx 和垂线方程 y = k_perp(x - x0) + y0
    // 设直线 y = kx，垂线 y - y0 = k_perp(x - x0)
    // 交点的 x 坐标
    float x_intersect = (k * x + y) / (k + k_perp);
    
    // 交点的 y 坐标
    float y_intersect = k * x_intersect;
    
    // 返回交点坐标作为 Eigen::Vector2f
    return Eigen::Vector2f(x_intersect, y_intersect);
}