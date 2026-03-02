#include "mpc_ibvs_controllerV3.hpp"
#include <ros/ros.h>
#include <iostream>
#include <new>
#include <algorithm>

// 构造函数实现
MPC_IBVS_Controller::MPC_IBVS_Controller(const MPCConfig& config) 
    : config_(config)
    , model_updated_(false)
{
    ROS_INFO("[MPC] Constructor called");
    
    // 1. 初始化固定大小矩阵
    A_ = Eigen::Matrix<double, 8, 8>::Identity();
    B_ = Eigen::Matrix<double, 8, 6>::Zero();
    
    // 2. 验证参数
    if (config_.Np <= 0) {
        ROS_WARN("[MPC] Invalid Np=%d, setting to default 6", config_.Np);
        config_.Np = 6;
    }
    
    int Np = config_.Np;
    ROS_INFO("[MPC] Np = %d", Np);
    
    // 3. 预分配动态矩阵
    try {
        // 先计算大小
        int rows = 8 * Np;
        int cols_s = 8;
        int cols_tau = 6 * Np;
        
        ROS_INFO("[MPC] Allocating matrices: M_s_(%d,%d), M_tau_(%d,%d)", 
                 rows, cols_s, rows, cols_tau);
        
        // 分配内存
        M_s_.resize(rows, cols_s);
        M_tau_.resize(rows, cols_tau);
        
        // 初始化为0
        M_s_.setZero();
        M_tau_.setZero();
        
        ROS_INFO("[MPC] Memory allocation successful");
        
    } catch (const std::bad_alloc& e) {
        ROS_ERROR("[MPC] Memory allocation failed: %s", e.what());
        throw;
    }
    
    ROS_INFO("[MPC] Controller initialized successfully");
}

// 析构函数
MPC_IBVS_Controller::~MPC_IBVS_Controller() {
    ROS_INFO("[MPC] Destructor called");
}

void MPC_IBVS_Controller::updateModel(const std::vector<vpFeaturePoint>& p_current,
                                     const std::vector<vpFeaturePoint>& pd_target) {
    if (p_current.size() < 4) return;

    Eigen::Matrix<double, 8, 6> L = Eigen::Matrix<double, 8, 6>::Zero();
    for (size_t i = 0; i < 4; ++i) {
        double x = p_current[i].get_x();
        double y = p_current[i].get_y();
        double Z = p_current[i].get_Z();
        
        if (Z <= 0.01) Z = 0.01; // 防止深度为0导致除零错误

        int r = 2 * i;
        L(r, 0) = -1.0 / Z;   L(r, 1) = 0;           L(r, 2) = x / Z;
        L(r, 3) = x * y;      L(r, 4) = -(1 + x*x);  L(r, 5) = y;

        L(r+1, 0) = 0;        L(r+1, 1) = -1.0 / Z; L(r+1, 2) = y / Z;
        L(r+1, 3) = 1 + y*y;  L(r+1, 4) = -x * y;    L(r+1, 5) = -x;
    }
    B_ = config_.Ts * L;
}

Eigen::VectorXd MPC_IBVS_Controller::solveMPC(const std::vector<vpFeaturePoint>& p_current,
                                             const std::vector<vpFeaturePoint>& pd_target,
                                             const Eigen::VectorXd& tau_prev) {
    if (p_current.size() < 4 || pd_target.size() < 4) return Eigen::VectorXd::Zero(6);

    updateModel(p_current, pd_target);
    buildPredictionMatrices();

    Eigen::VectorXd s_curr = featuresToState(p_current);
    Eigen::VectorXd s_targ = featuresToState(pd_target);

    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    computeQPMatricesDense(s_curr, s_targ, H, f);

    // 使用支持鲁棒求解的矩阵分解
    Eigen::LDLT<Eigen::MatrixXd> solver(H);
    if (solver.info() != Eigen::Success) {
        ROS_ERROR("MPC QP Solver failed!");
        return Eigen::VectorXd::Zero(6);
    }

    Eigen::VectorXd U = solver.solve(-f);
    Eigen::VectorXd cmd = U.head(6);

    // 应用约束
    applyVelocityConstraints(cmd);
    applyFOVConstraints(p_current, cmd);

    return cmd;
}

void MPC_IBVS_Controller::applyVelocityConstraints(Eigen::VectorXd& v_c) const {
    // 线性速度约束
    for(int i = 0; i < 3; ++i) {
        v_c(i) = std::max(std::min(v_c(i), config_.v_max), -config_.v_max);
    }
    
    // 角速度约束
    for(int i = 3; i < 6; ++i) {
        v_c(i) = std::max(std::min(v_c(i), config_.w_max), -config_.w_max);
    }
    
    // 平滑约束：限制加速度变化
    static Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(6);
    double max_accel_linear = config_.v_max * 0.5;  // 最大加速度为最大速度的50%
    double max_accel_angular = config_.w_max * 0.5;
    
    for(int i = 0; i < 3; ++i) {
        double delta = v_c(i) - v_prev(i);
        delta = std::max(std::min(delta, max_accel_linear), -max_accel_linear);
        v_c(i) = v_prev(i) + delta;
    }
    
    for(int i = 3; i < 6; ++i) {
        double delta = v_c(i) - v_prev(i);
        delta = std::max(std::min(delta, max_accel_angular), -max_accel_angular);
        v_c(i) = v_prev(i) + delta;
    }
    
    v_prev = v_c;
}

void MPC_IBVS_Controller::applyFOVConstraints(const std::vector<vpFeaturePoint>& p_current,
                                            Eigen::VectorXd& v_c) const {
    bool near_boundary = false;
    
    // 检查所有特征点是否接近边界
    for (size_t i = 0; i < p_current.size(); ++i) {
        if (checkFOVViolation(p_current[i])) {
            near_boundary = true;
            break;
        }
    }
    
    if (near_boundary) {
        // 当特征点接近边界时，限制某些运动分量
        ROS_WARN_THROTTLE(1.0, "Applying FOV constraints");
        
        // 限制远离相机的运动（防止特征点移出视野）
        if (v_c[2] < 0) {  // 如果相机在远离目标
            v_c[2] *= 0.5;  // 减小远离速度
        }
        
        // 限制可能导致特征点移出视野的旋转运动
        v_c[3] *= 0.7;  // 减小绕x轴旋转
        v_c[4] *= 0.7;  // 减小绕y轴旋转
        
        // 增强朝向目标的运动
        if (v_c[2] > 0) {  // 如果相机在靠近目标
            v_c[2] *= 1.2;  // 稍微增加靠近速度
        }
    }
}

bool MPC_IBVS_Controller::checkFOVViolation(const vpFeaturePoint& feature) const {
    // 将特征点坐标转换为像素坐标
    double u = feature.get_x() * config_.fx + config_.cx;
    double v = feature.get_y() * config_.fy + config_.cy;
    
    // 检查是否接近图像边界
    return (u < config_.fov_margin || u > config_.image_width - config_.fov_margin ||
            v < config_.fov_margin || v > config_.image_height - config_.fov_margin);
}

void MPC_IBVS_Controller::buildPredictionMatrices() {
    int n = 8, m = 6, Np = config_.Np;
    M_s_.setZero();
    M_tau_.setZero();

    Eigen::MatrixXd A_pow = Eigen::MatrixXd::Identity(n, n);
    for (int i = 0; i < Np; ++i) {
        A_pow = A_pow * A_; // A^i
        M_s_.block(i * n, 0, n, n) = A_pow;

        for (int j = 0; j <= i; ++j) {
            // 这里假设模型在预测期内B保持不变
            M_tau_.block(i * n, j * m, n, m) = B_; 
        }
    }
}

void MPC_IBVS_Controller::computeQPMatricesDense(const Eigen::VectorXd& s_current,
                                               const Eigen::VectorXd& s_target,
                                               Eigen::MatrixXd& H,
                                               Eigen::VectorXd& f) const {
    int n = 8, m = 6, Np = config_.Np;
    Eigen::MatrixXd Q_big = Eigen::MatrixXd::Zero(n * Np, n * Np);
    Eigen::MatrixXd R_big = Eigen::MatrixXd::Zero(m * Np, m * Np);
    Eigen::VectorXd S_target_big = Eigen::VectorXd::Zero(n * Np);

    for (int i = 0; i < Np; ++i) {
        Q_big.block(i * n, i * n, n, n) = config_.Q;
        R_big.block(i * m, i * m, m, m) = config_.R;
        S_target_big.segment(i * n, n) = s_target;
    }
    
    H = 2.0 * (M_tau_.transpose() * Q_big * M_tau_ + R_big);
    f = 2.0 * M_tau_.transpose() * Q_big * (M_s_ * s_current - S_target_big);
}

Eigen::VectorXd MPC_IBVS_Controller::featuresToState(const std::vector<vpFeaturePoint>& features) const {
    Eigen::VectorXd s(8);
    for (int i = 0; i < 4; ++i) {
        s(2 * i) = features[i].get_x();
        s(2 * i + 1) = features[i].get_y();
    }
    return s;
}
