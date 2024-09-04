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
    float x_intersect = (y - k_perp * x) / (k - k_perp);
    
    // 交点的 y 坐标
    float y_intersect = k * x_intersect;
    
    // 返回交点坐标作为 Eigen::Vector2f
    return Eigen::Vector2f(x_intersect, y_intersect);
}

double normalizeAngle(double angle, float angle_thershold) {
    while (angle > angle_thershold) {
        angle = angle_thershold;
    }
    while (angle < -angle_thershold) {
        angle = -angle_thershold;
    }
    return angle;
}

bool isPotentialConflict(const Eigen::Vector3f& positionA, const Eigen::Vector3f& velocityA, const Eigen::Vector3f& positionB, const Eigen::Vector3f& velocityB, const float& psi_therahold, const float& theta_therahold, const float& potential_T, const int& sample_num, const float& safe_therahold){
    auto [psi_i_now, theta_i_now] = Velocity2Angles(velocityA);
    auto [psi_j_now, theta_j_now] = Velocity2Angles(velocityB);
    std::vector<Eigen::Vector3f> relative_poses;
    for(int i = -sample_num; i <=sample_num; i++){
        double psi_i_sample = psi_i_now + i * (psi_therahold / (2*sample_num+1));
        psi_i_sample = normalizeAngle(psi_i_sample, M_PI);
        double theta_i_sample = theta_i_now + i * (theta_therahold / (2*sample_num)+1);
        theta_i_sample = normalizeAngle(theta_i_sample, 0.5 * M_PI);
        Eigen::Vector3f vel_i_sample = Angles2Velocity(psi_i_now, theta_i_now, velocityA.norm());

        for(int j = -sample_num; j <=sample_num; j++){
            double psi_j_sample = psi_j_now + j * (psi_therahold / (2*sample_num+1));
            psi_j_sample = normalizeAngle(psi_j_sample, M_PI);
            double theta_j_sample = theta_j_now + j * (theta_therahold / (2*sample_num+1));
            theta_j_sample = normalizeAngle(theta_j_sample, 0.5 * M_PI);
            Eigen::Vector3f vel_j_sample = Angles2Velocity(psi_j_now, theta_j_now, velocityB.norm());

            Eigen::Vector3f relativePos_sample = (vel_i_sample - vel_j_sample) * potential_T;
            relative_poses.push_back(relativePos_sample);
        }
    }
    Eigen::Vector3f max_point, min_point;
    Eigen::Matrix3f R;
    std::tie(max_point, min_point, R) = maxV_point(relative_poses, safe_therahold);
    bool inside_flag = checkrealpoint(positionA, positionB, min_point, max_point, R);


    
return inside_flag;
}

std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Matrix3f> maxV_point(const std::vector<Eigen::Vector3f>& relative_poses, const float& safe_therahold){
    float V_min = std::numeric_limits<float>::infinity();
    Eigen::Matrix3f R_min;
    Eigen::Vector3f max_point, min_point ;
    
    for(int i=-6; i<=6; i++){
        for(int j=-3; j<=3; j++){
            float new_yaw = i*(M_PI/6);
            float new_pitch = j*(M_PI/6);
            Eigen::Matrix3f rx, rz;
            rx << 1, 0, 0, 
                0, cos(new_pitch), -sin(new_pitch), 
                0, sin(new_pitch), cos(new_pitch);
            rz << cos(new_yaw), -sin(new_yaw), 0,
                0, sin(new_yaw), cos(new_yaw),
                0, 0, 1;
            Eigen::Matrix3f R = rx * rz;

            if (!relative_poses.empty()) {
                
                Eigen::Vector3f max_point_, min_point_ ;

                // Initialize min and max values with the first element
                Eigen::Vector3f safe_therahold_;
                safe_therahold_ << safe_therahold, safe_therahold, safe_therahold;

                min_point_.x() = (R * (relative_poses[0])-safe_therahold_).x();
                max_point_.x() = (R * (relative_poses[0])+safe_therahold_).x();
                min_point_.y() = (R * (relative_poses[0])-safe_therahold_).y();
                max_point_.y() = (R * (relative_poses[0])+safe_therahold_).y();
                min_point_.z() = (R * (relative_poses[0])-safe_therahold_).z();
                max_point_.z() = (R * (relative_poses[0])+safe_therahold_).z();
                // Iterate over the relative_poses to find min and max values
                for (const auto& pos : relative_poses) {

                    // Update x min/max
                    if ((R*(pos)-safe_therahold_).x() < min_point_.x()) min_point_.x() = (R*(pos)-safe_therahold_).x();
                    if ((R*(pos)+safe_therahold_).x() > max_point_.x()) max_point_.x() = (R*(pos)+safe_therahold_).x();

                    // Update y min/max
                    if ((R*(pos)-safe_therahold_).y() < min_point_.y()) min_point_.y() = (R*(pos)-safe_therahold_).y();
                    if ((R*(pos)+safe_therahold_).y() > max_point_.y()) max_point_.y() = (R*(pos)+safe_therahold_).y();

                    // Update z min/max
                    if ((R*(pos)-safe_therahold_).z() < min_point_.z()) min_point_.z() = (R*(pos)-safe_therahold_).z();
                    if ((R*(pos)+safe_therahold_).z() > max_point_.z()) max_point_.z() = (R*(pos)+safe_therahold_).z();
                }
                // 计算体积
                float volume = (max_point_.x() - min_point_.x()) * (max_point_.y() - min_point_.y()) * (max_point_.z() - min_point_.z());
                if(volume < V_min){
                    R_min = R;
                    V_min = volume;
                    max_point = R.inverse() * max_point_;
                    min_point = R.inverse() * min_point_;
                    
                }

            }

        }
    }

    return std::make_tuple(max_point, min_point, R_min);
}

// 计算由四个点确定的四面体的体积
float TetrahedronVolume(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3, const Eigen::Vector3f& p4) {
    return std::fabs((p2 - p1).dot((p3 - p1).cross(p4 - p1))) / 6.0f;
}

// 判断点A是否在四棱锥内
bool IsPointInPyramid(const Eigen::Vector3f& apex, const Eigen::Vector3f& base1, const Eigen::Vector3f& base2, const Eigen::Vector3f& base3, const Eigen::Vector3f& base4, const Eigen::Vector3f& A) {
    // 原四棱锥的体积
    float original_volume = TetrahedronVolume(apex, base1, base2, base3) 
                          + TetrahedronVolume(apex, base1, base3, base4);

    // 计算点A与底面各三角形组成的四面体的体积
    float volume1 = TetrahedronVolume(A, base1, base2, base3);
    float volume2 = TetrahedronVolume(A, base1, base3, base4);
    float volume3 = TetrahedronVolume(A, apex, base1, base2);
    float volume4 = TetrahedronVolume(A, apex, base2, base3);
    float volume5 = TetrahedronVolume(A, apex, base3, base4);
    float volume6 = TetrahedronVolume(A, apex, base4, base1);

    // 如果所有小四面体的体积之和等于原四棱锥的体积，则A在四棱锥内
    float total_volume = volume1 + volume2 + volume3 + volume4 + volume5 + volume6;
    return std::fabs(total_volume - original_volume) < 1e-6;  // 考虑浮点误差
}


// 判断点P是否在三棱锥（四面体）内部
bool IsPointInTetrahedron(const Eigen::Vector3f& P, const Eigen::Vector3f& A, const Eigen::Vector3f& B, const Eigen::Vector3f& C, const Eigen::Vector3f& D) {
    // 计算原始四面体ABCD的体积
    float original_volume = TetrahedronVolume(A, B, C, D);

    // 计算点P和每个面形成的小四面体的体积
    float volume1 = TetrahedronVolume(P, B, C, D);
    float volume2 = TetrahedronVolume(A, P, C, D);
    float volume3 = TetrahedronVolume(A, B, P, D);
    float volume4 = TetrahedronVolume(A, B, C, P);

    // 如果子四面体的体积之和等于原四面体的体积，并且所有体积都为正，则点P在三棱锥内
    float total_volume = volume1 + volume2 + volume3 + volume4;
    if (std::fabs(total_volume - original_volume) < 1e-6) { // 考虑浮点数误差
        return true;
    }
    return false;
}


bool checkrealpoint(const Eigen::Vector3f& self_point,const Eigen::Vector3f& conflict_point, const Eigen::Vector3f& min_point, const Eigen::Vector3f& max_point, const Eigen::Matrix3f& R){
    // Eigen::Vector3f self_point_ = R * self_point;
    Eigen::Vector3f j_point_ = R * (conflict_point - self_point);
    Eigen::Vector3f min_point_ = R * min_point;
    Eigen::Vector3f max_point_ = R * max_point;
    Eigen::Vector3f self_point_;
    self_point_ << 0, 0, 0;

    Eigen::Vector3f point1, point2, point3, point4, point5, point6, point7, point8;
    point1 << min_point.x(), min_point.y(), min_point.z();
    point2 << max_point.x(), min_point.y(), min_point.z();
    point3 << max_point.x(), min_point.y(), max_point.z();
    point4 << min_point.x(), min_point.y(), max_point.z();
    point5 << min_point.x(), max_point.y(), min_point.z();
    point6 << max_point.x(), max_point.y(), min_point.z();
    point7 << max_point.x(), max_point.y(), max_point.z();
    point8 << min_point.x(), max_point.y(), max_point.z();

    //     z
    //     ^ 8----7
    //     |/    /| /
    //     5----6 3/
    //     | /  | /
    //  ___1----2/____>x
    //     |    |
    //总共将空间分为27份，除去自己中心，剩下26份
    //分为3类：
    //      1、在点上的8个空间：用与点相邻的三个点构成四面体
    //      2、在边上的12个空间：用对应的斜面构成四棱锥
    //      3、在面上的6个空间：直接用面构成四棱锥
    if(min_point_.x() <= self_point_.x() && self_point_.x() <= max_point_.x() && min_point_.y() <= self_point_.y() && self_point_.y() <= max_point_.y() && min_point_.z() <= self_point_.z() && self_point_.z() <= max_point_.z()){
        std::cout << "**** 在长方体内 *****" << std::endl;
        
        return true;

    }
    else if(self_point_.x() <= min_point_.x() && self_point_.y() <= min_point_.y() && self_point_.z() <= min_point_.z()){
        return IsPointInTetrahedron(j_point_, self_point_, point2, point4, point5);//1
    }else if(self_point_.x() > max_point_.x() && self_point_.y() <= min_point_.y() && self_point_.z() <= min_point_.z()){
        return IsPointInTetrahedron(j_point_, self_point_, point1, point3, point6);//2
    }else if(self_point_.x() > max_point_.x() && self_point_.y() > max_point_.y() && self_point_.z() <= min_point_.z()){
        return IsPointInTetrahedron(j_point_, self_point_, point2, point4, point7);//3
    }else if(self_point_.x() <= min_point_.x() && self_point_.y() > max_point_.y() && self_point_.z() <= min_point_.z()){
        return IsPointInTetrahedron(j_point_, self_point_, point1, point3, point8);//4
    }else if(self_point_.x() <= min_point_.x() && self_point_.y() <= min_point_.y() && self_point_.z() > max_point_.z()){
        return IsPointInTetrahedron(j_point_, self_point_, point1, point6, point8);//5
    }else if(self_point_.x() > max_point_.x() && self_point_.y() <= min_point_.y() && self_point_.z() > max_point_.z()){
        return IsPointInTetrahedron(j_point_, self_point_, point2, point5, point7);//6
    }else if(self_point_.x() > max_point_.x() && self_point_.y() > max_point_.y() && self_point_.z() > max_point_.z()){
        return IsPointInTetrahedron(j_point_, self_point_, point3, point6, point8);//7
    }else if(self_point_.x() <= min_point_.x() && self_point_.y() > max_point_.y() && self_point_.z() > max_point_.z()){
        return IsPointInTetrahedron(j_point_, self_point_, point4, point5, point7);//8
    }
    else if(min_point_.x() <= self_point_.x() && self_point_.x() <= max_point_.x() && self_point_.y() <= min_point_.y() && self_point_.z() <= min_point_.z()){
        return IsPointInPyramid(self_point_, point3, point4, point5, point6, j_point_);//12
    }else if(self_point_.x() > max_point_.x() && min_point.y() <= self_point_.y() && self_point_.y() <= max_point_.y() && self_point_.z() <= min_point_.z()){
        return IsPointInPyramid(self_point_, point1, point4, point6, point7, j_point_);//23
    }else if(min_point_.x() <= self_point_.x() && self_point_.x() <= max_point_.x() && self_point_.y() > max_point_.y() && self_point_.z() <= min_point_.z()){
        return IsPointInPyramid(self_point_, point1, point2, point7, point8, j_point_);//34
    }else if(self_point_.x() <= min_point_.x() && min_point.y() <= self_point_.y() && self_point_.y() <= max_point_.y() && self_point_.z() <= min_point_.z()){
        return IsPointInPyramid(self_point_, point2, point3, point5, point8, j_point_);//41
    }else if(min_point_.x() <= self_point_.x() && self_point_.x() <= max_point_.x() && self_point_.y() <= min_point_.y() && self_point_.z() > max_point_.z()){
        return IsPointInPyramid(self_point_, point1, point2, point7, point8, j_point_);//56
    }else if(self_point_.x() > max_point_.x() && min_point.y() <= self_point_.y() && self_point_.y() <= max_point_.y() && self_point_.z() > max_point_.z()){
        return IsPointInPyramid(self_point_, point2, point3, point5, point8, j_point_);//67
    }else if(min_point_.x() <= self_point_.x() && self_point_.x() <= max_point_.x() && self_point_.y() > max_point_.y() && self_point_.z() > max_point_.z()){
        return IsPointInPyramid(self_point_, point3, point4, point5, point6, j_point_);//78
    }else if(self_point_.x() <= min_point_.x() && min_point.y() <= self_point_.y() && self_point_.y() <= max_point_.y() && self_point_.z() > max_point_.z()){
        return IsPointInPyramid(self_point_, point1, point4, point6, point7, j_point_);//85
    }else if(self_point_.x() <= min_point_.x() && self_point_.y() <= min_point_.y() && min_point_.z() <= self_point_.z() && self_point_.z() <= max_point_.z()){
        return IsPointInPyramid(self_point_, point2, point4, point6, point8, j_point_);//15
    }else if(self_point_.x() > max_point_.x() && self_point_.y() <= min_point_.y() && min_point_.z() <= self_point_.z() && self_point_.z() <= max_point_.z()){
        return IsPointInPyramid(self_point_, point1, point3, point5, point7, j_point_);//26
    }else if(self_point_.x() > max_point_.x() && self_point_.y() > max_point_.y() && min_point_.z() <= self_point_.z() && self_point_.z() <= max_point_.z()){
        return IsPointInPyramid(self_point_, point2, point4, point6, point8, j_point_);//37
    }else if(self_point_.x() <= min_point_.x() && self_point_.y() > max_point_.y() && min_point_.z() <= self_point_.z() && self_point_.z() <= max_point_.z()){
        return IsPointInPyramid(self_point_, point1, point3, point5, point7, j_point_);//48
    }
    else if(min_point_.x() <= self_point_.x() && self_point_.x() <= max_point_.x() && self_point_.z() <= min_point_.z() && min_point_.y() <= self_point_.y() && self_point_.y() <= max_point_.y()){
        return IsPointInPyramid(self_point_, point1, point2, point3, point4, j_point_);//1234
    }else if(min_point_.x() <= self_point_.x() && self_point_.x() <= max_point_.x() && min_point_.z() <= self_point_.z() && self_point_.z() <= max_point_.z() && self_point_.y() <= min_point_.y()){
        return IsPointInPyramid(self_point_, point1, point2, point5, point6, j_point_);//1256
    }else if(self_point_.x() > max_point_.x() && min_point_.y() <= self_point_.y() && self_point_.y() <= max_point_.y() && min_point_.z() <= self_point_.z() && self_point_.z() <= max_point_.z()){
        return IsPointInPyramid(self_point_, point2, point3, point6, point7, j_point_);//2367
    }else if(min_point_.x() <= self_point_.x() && self_point_.x() <= max_point_.x() && min_point_.z() <= self_point_.z() && self_point_.z() <= max_point_.z() && self_point_.y() > max_point_.y()){
        return IsPointInPyramid(self_point_, point3, point4, point7, point8, j_point_);//3478
    }else if(self_point_.x() <= min_point_.x() && min_point_.y() <= self_point_.y() && self_point_.y() <= max_point_.y() && min_point_.z() <= self_point_.z() && self_point_.z() <= max_point_.z()){
        return IsPointInPyramid(self_point_, point1, point2, point5, point6, j_point_);//1458
    }else if(min_point_.x() <= self_point_.x() && self_point_.x() <= max_point_.x() && self_point_.z() > max_point_.z() && min_point_.y() <= self_point_.y() && self_point_.y() <= max_point_.y()){
        return IsPointInPyramid(self_point_, point5, point6, point7, point8, j_point_);//5678
    }
    else{
        std::cout <<"========================================================"<<std::endl;
        std::cout <<""<<std::endl;
        std::cout <<""<<std::endl;
        std::cout << "######################  发生错误  ######################" << std::endl;
        std::cout <<""<<std::endl;
        std::cout <<""<<std::endl;
        std::cout <<"========================================================"<<std::endl;
        return 0;
    }

}