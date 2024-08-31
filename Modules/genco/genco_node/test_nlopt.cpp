#include <iostream>
#include <nlopt.hpp>
#include <Eigen/Dense>
#include <tuple>

std::tuple<double, double> Velocity2Angles(const Eigen::Vector3f& V) ;
Eigen::Vector3f Angles2Velocity(const double& psi, const double& theta, const double& V);
std::tuple<Eigen::Vector3f, Eigen::Vector3f, float, Eigen::Matrix3f> RelativeVelocity2NewFrame(const Eigen::Vector3f& positionA, const Eigen::Vector3f& velocityA, const Eigen::Vector3f& positionB, const Eigen::Vector3f& velocityB, const float& safe_therahold ) ;
Eigen::Vector2f calculatePerpendicularIntersection(float x, float y, float k) ;
Eigen::Matrix3f computeRotationMatrix(const float& alpha, const float& beta) ;
float calculateSlope(const Eigen::Vector3f& velocity) ;






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

// 定义一个函数来计算垂足的交点坐标，并返回 Eigen::Vector2f
Eigen::Vector2f calculatePerpendicularIntersection(float x, float y, float k) {
    // 计算垂线斜率 k_perp = -1/k
    float k_perp = -1.0f / k;
    
    // 使用直线方程 y = kx 和垂线方程 y = k_perp(x - x0) + y0
    // 设直线 y = kx，垂线 y - y0 = k_perp(x - x0)
    // 交点的 x 坐标
    // float x_intersect = (k_perp * x + y) / (k + k_perp);
    float x_intersect = (y - k_perp * x) / (k - k_perp);
    
    // 交点的 y 坐标
    float y_intersect = k * x_intersect;
    
    // 返回交点坐标作为 Eigen::Vector2f
    return Eigen::Vector2f(x_intersect, y_intersect);
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
//=============================================================================================================
//=============================================================================================================
//=============================================================================================================
//=============================================================================================================
//=============================================================================================================
//=============================================================================================================

// 目标函数，接受一个参数向量并返回目标值
double objective_function(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
    // 计算目标函数的值
    double result = 0.0;

    double psi_i = x[0];
    double theta_i = x[1];
    result = std::pow((psi_i - (-2.36)), 2) + std::pow((theta_i - (0.48)), 2);
   
    return result;
}

void constraint_function(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data) {
    
    double psi_i = x[0];
    double theta_i = x[1];

   result[0] = psi_i - (-3.14);  // psi_i >= psi_min
    result[1] = (3.14) - psi_i;  // psi_i <= psi_max
    result[2] = theta_i - (-1.57);  // theta_i >= theta_min
    result[3] = (1.57) - theta_i;  // theta_i <= theta_max

    // 添加非线性约束
    Eigen::Vector3f V_i = {-0.49, -0.49, 0.36};
    Eigen::Vector3f P_i = {8.64, 7.90, 19.74};
    Eigen::Vector3f V_j = {0.52, 0.45, -0.38};
    Eigen::Vector3f P_j = {-7.77, -8.67, 30.65};
    Eigen::Vector3f Vel_, Pos_;
    float therahold_slope;
    Eigen::Matrix3f R;
    std::tie(Pos_, Vel_, therahold_slope, R) = RelativeVelocity2NewFrame(P_i , V_i, P_j, V_j, 5.0);

    Eigen::Vector3f V_i_ = R * V_i;
    Eigen::Vector3f V_j_ = R * V_j;
    auto[psi_i_, theta_i_] = Velocity2Angles(V_i_);
    auto[psi_j_, theta_j_] = Velocity2Angles(V_j_);
    float X_ = cos(psi_j_)*cos(theta_j_) - sin(theta_j_) / therahold_slope;
    float Y_ = cos(psi_i_)*cos(theta_i_) - sin(theta_i_) / therahold_slope;
    float K_ = V_j_.norm() / V_i_.norm();

    Eigen::Vector2f target_xy_ = calculatePerpendicularIntersection(X_, Y_, K_);

    // std::cout << "X = " << X_ <<" Y = " << Y_ << " K = " << K_<< "  target_x = " << target_xy_[0] << "target_y = " << target_xy_[1] << std::endl;

    result[4] = Y_ - target_xy_[1];
   // 检查结果数组内容
    std::cout << "Constraints: ";
    for (unsigned i = 0; i < m; ++i) {
        std::cout << result[i] << " ";
    }
    std::cout << std::endl;
        }


int main() {
    // 创建优化器对象，选择优化算法
    nlopt::opt opt;


    

    //  opt = nlopt::opt(nlopt::LD_SLSQP, 2); // 选择全局优化算法
     opt = nlopt::opt(nlopt::GN_ISRES, 2); // 选择全局优化算法
    // opt = nlopt::opt(nlopt::LD_MMA, 2);  // 选择 GN_AGS 算法，2 代表变量个数


     
    // opt = nlopt::opt(nlopt::GN_CRS2_LM, 2); // 选择全局优化算法
    // nlopt::opt local_opt(nlopt::LD_SLSQP, 2); // 选择局部优化算法
    // opt.set_local_optimizer(local_opt);


    // 设置目标函数
    opt.set_min_objective(objective_function, nullptr);

    // 设置约束
    std::vector<double> tol(5, 1e-8);  // 容差向量


    // 设置初始点
    Eigen::Vector3f V_i = {-0.49, 0.49, 0.36};
    Eigen::Vector3f P_i = {8.64, 7.90, 19.74};
    Eigen::Vector3f V_j = {0.52, 0.45, -0.38};
    Eigen::Vector3f P_j = {-7.77, -8.67, 30.65};
    auto [psi_now, theta_now] = Velocity2Angles(V_i);
    std::vector<double> x={psi_now, theta_now};  // 初始点
     // 输出初始点值
    std::cout << "Initial psi_now: " << psi_now << std::endl;
    std::cout << "Initial theta_now: " << theta_now << std::endl;

    opt.add_inequality_mconstraint(constraint_function, nullptr, tol);







   

    


    

    // 设置变量的边界
    std::vector<double> lb{-3.14, -1.57};  // 下边界
    std::vector<double> ub{3.14, 1.57};  // 上边界
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    // 设置优化参数
    opt.set_xtol_rel(1e-6);  // 设置相对容忍度

    
    opt.set_maxeval(1000); 

    // 执行优化
    double minf;  // 最小化后的目标值
    try {
        nlopt::result result = opt.optimize(x, minf);

        std::cout << "Optimal x: ";
        for (size_t i = 0; i < x.size(); ++i) {
            std::cout << x[i] << " ";
        }
        std::cout << std::endl;

        std::cout << "Minimum value: " << minf << std::endl;
    } catch (std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}

