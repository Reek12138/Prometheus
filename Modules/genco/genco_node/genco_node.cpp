#include "genco_node.h"
#include <Eigen/Dense>
#include <cmath>
#include <tuple>

// 计算无人机A在以B为z轴且静止的坐标系下的相对速度,返回无人机B的坐标，无人机a的速度，安全斜率
std::tuple<Eigen::Vector3f, Eigen::Vector3f, float> RelativeVelocity2NewFrame(const Eigen::Vector3f& positionA, const Eigen::Vector3f& velocityA, const Eigen::Vector3f& positionB, const Eigen::Vector3f& velocityB, const float& safe_therahold ) {
    // 计算相对位置和速度
    Eigen::Vector3f relativePosition = positionB - positionA;
    Eigen::Vector3f relativeVelocity = velocityA - velocityB;

    // 定义新坐标系的 z 轴方向
    Eigen::Vector3f zAxisNewFrame = relativePosition.normalized();

    // 原始坐标系的 z 轴方向
    Eigen::Vector3f zAxisOriginal(0, 0, 1);

    // 计算旋转矩阵
    Eigen::Matrix3f rotationMatrix = computeRotationMatrix(zAxisOriginal, zAxisNewFrame);

    // 旋转相对速度到新坐标系
    Eigen::Vector3f vel = rotationMatrix * relativeVelocity;
    Eigen::Vector3f pos = rotationMatrix * relativePosition;

    float c_ = relativePosition.norm();
    float b_ = std::sqrt(c_*c_ - safe_therahold*safe_therahold);
    float slope = b_/safe_therahold;

    return std::make_tuple(pos, vel, slope);
    }


// 计算旋转矩阵，从一个向量到另一个向量
// R=I+sin(θ)⋅K+(1−cos(θ))⋅K*2
// Eigen::Matrix3f computeRotationMatrix(const Eigen::Vector3f& from, const Eigen::Vector3f& to) {
//     Eigen::Vector3f v = from.cross(to);
//     float s = v.norm();
//     float c = from.dot(to);

//     Eigen::Matrix3f vx;
//     vx << 0, -v.z(), v.y(),
//           v.z(), 0, -v.x(),
//           -v.y(), v.x(), 0;

//     Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

//     Eigen::Matrix3f rotationMatrix;
//     if (s == 0) {
//         // from 和 to 是平行的或反平行的
//         if (c > 0) {
//             // from 和 to 是平行的
//             rotationMatrix = I;
//         } else {
//             // from 和 to 是反平行的
//             Eigen::Vector3f perp;
//             if (std::abs(from.x()) <= std::abs(from.y()) && std::abs(from.x()) <= std::abs(from.z())) {
//                 perp = Eigen::Vector3f(0, -from.z(), from.y());
//             } else if (std::abs(from.y()) <= std::abs(from.x()) && std::abs(from.y()) <= std::abs(from.z())) {
//                 perp = Eigen::Vector3f(-from.z(), 0, from.x());
//             } else {
//                 perp = Eigen::Vector3f(-from.y(), from.x(), 0);
//             }
//             perp.normalize();
//             rotationMatrix = 2 * perp * perp.transpose() - I;
//         }
//     } else {
//         // 使用罗德里格斯旋转公式
//         rotationMatrix = I + vx + vx * vx * ((1 - c) / (s * s));
//     }

//     return rotationMatrix;
// }
Eigen::Matrix3f computeRotationMatrix(const Eigen::Vector3f& zAxisOriginal, const Eigen::Vector3f& zAxisNew) {
    // 计算旋转轴 (cross product)
    Eigen::Vector3f rotationAxis = zAxisOriginal.cross(zAxisNew).normalized();
    
    // 计算旋转角度 (dot product)
    float angle = std::acos(zAxisOriginal.dot(zAxisNew) / (zAxisOriginal.norm() * zAxisNew.norm()));

    // 特殊情况处理：如果旋转轴的范数为0（即两个向量平行或反平行）
    if (rotationAxis.norm() == 0) {
        if (zAxisOriginal.dot(zAxisNew) > 0) {
            return Eigen::Matrix3f::Identity(); // 平行，返回单位矩阵
        } else {
            // 反平行，选择一个垂直于z轴的任意轴进行180度旋转
            rotationAxis = zAxisOriginal.unitOrthogonal();
            angle = M_PI;
        }
    }

    // 计算旋转矩阵 (Rodrigues' rotation formula)
    Eigen::Matrix3f rotationMatrix;
    Eigen::Matrix3f K;
    K << 0, -rotationAxis.z(), rotationAxis.y(),
         rotationAxis.z(), 0, -rotationAxis.x(),
         -rotationAxis.y(), rotationAxis.x(), 0;

    rotationMatrix = Eigen::Matrix3f::Identity() + std::sin(angle) * K + (1 - std::cos(angle)) * K * K;

    return rotationMatrix;
}




// 生成轨迹
std::vector<TrajectoryPoint> generateTrajectory(const Eigen::Vector3f& start, const Eigen::Vector3f& end, float total_time, const std::vector<float>& time_intervals) {
    Eigen::Vector3f velocity = (end - start) / total_time;
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

