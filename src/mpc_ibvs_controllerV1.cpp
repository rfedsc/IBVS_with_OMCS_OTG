#include "mpc_ibvs_controllerV1.hpp"
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <ros/ros.h>

MPC_IBVS_Controller::MPC_IBVS_Controller(const MPCConfig& config) 
    : config_(config), 
      solver_initialized_(false),
      model_updated_(false) {
    
    A_ = Eigen::Matrix<double, 8, 8>::Identity();
    B_ = Eigen::Matrix<double, 8, 6>::Zero();
    
    computeFOVBounds();
    
    ROS_INFO("MPC_IBVS_Controller initialized with interaction type: %d", 
             static_cast<int>(config_.interaction_type));
}

void MPC_IBVS_Controller::setConfig(const MPCConfig& config) {
    config_ = config;
    computeFOVBounds();
    solver_initialized_ = false;
    model_updated_ = false;
    
    ROS_INFO("MPC configuration updated");
}

void MPC_IBVS_Controller::updateModel(
    const std::vector<vpFeaturePoint>& p_current,
    const std::vector<vpFeaturePoint>& pd_target)
{
    if (p_current.size() != 4 || pd_target.size() != 4) {
        throw std::invalid_argument("Need exactly 4 current and 4 target features");
    }

    p_current_.clear();
    pd_target_.clear();
    p_current_.reserve(4);
    pd_target_.reserve(4);

    // 8x6 interaction matrix
    Eigen::Matrix<double, 8, 6> L;
    L.setZero();

    for (size_t i = 0; i < 4; ++i) {
        vpFeaturePoint s  = p_current[i];
        vpFeaturePoint sd = pd_target[i];

        double Z = s.get_Z();
        if (Z <= 0 || std::isnan(Z)) {
            ROS_ERROR("Invalid depth at feature %zu: Z=%f", i, Z);
            return;
        }

        //图像归一化坐标
        double x = s.get_x();
        double y = s.get_y();

        s.set_Z(Z);
        sd.set_Z(sd.get_Z());

        p_current_.push_back(s);
        pd_target_.push_back(sd);

        // --- 手写 IBVS interaction matrix ---
        const int r = 2 * i;

        L(r, 0) = -1.0 / Z;
        L(r, 1) =  0.0;
        L(r, 2) =  x / Z;
        L(r, 3) =  x * y;
        L(r, 4) = -(1.0 + x * x);
        L(r, 5) =  y;

        L(r+1, 0) =  0.0;
        L(r+1, 1) = -1.0 / Z;
        L(r+1, 2) =  y / Z;
        L(r+1, 3) =  1.0 + y * y;
        L(r+1, 4) = -x * y;
        L(r+1, 5) = -x;
    }

    // MPC 离散模型
    B_ = config_.Ts * L;
    A_.setIdentity();

    model_updated_ = true;
}


Eigen::Matrix<double, 8, 6> MPC_IBVS_Controller::convertVispToEigen(const vpMatrix& L_visp) const {
    if (L_visp.getRows() != 8 || L_visp.getCols() != 6) {
        ROS_ERROR("ViSP interaction matrix must be 8x6, got %ux%u", 
                  (unsigned int)L_visp.getRows(), (unsigned int)L_visp.getCols());
        throw std::runtime_error("ViSP interaction matrix must be 8x6");
    }
    
    Eigen::Matrix<double, 8, 6> L_eigen;
    
    for (unsigned int i = 0; i < 8; i++) {
        for (unsigned int j = 0; j < 6; j++) {
            L_eigen(i, j) = L_visp[i][j];
        }
    }
    
    return L_eigen;
}


Eigen::Matrix<double, 8, 6> MPC_IBVS_Controller::computeInteractionMatrixFromFeaturesOld(
    const std::vector<vpFeaturePoint>& p_current) {
    
    if (p_current.size() != 4) {
        throw std::invalid_argument("Need exactly 4 feature points");
    }
    
    Eigen::Matrix<double, 8, 6> Ls = Eigen::Matrix<double, 8, 6>::Zero();
    
    for (int i = 0; i < 4; i++) {
        // 创建特征的副本（可修改）
        vpFeaturePoint feature_copy = p_current[i];
        
        // 在副本上调用interaction()方法
        vpMatrix L_i = feature_copy.interaction();
        
        for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 6; col++) {
                Ls(2 * i + row, col) = L_i[row][col];
            }
        }
    }
    
    return Ls;
}

Eigen::VectorXd MPC_IBVS_Controller::featuresToState(const std::vector<vpFeaturePoint>& features) const {
    if (features.size() != 4) {
        throw std::invalid_argument("Need exactly 4 feature points");
    }
    
    Eigen::VectorXd s(8);
    for (size_t i = 0; i < 4; i++) {
        s(2 * i) = features[i].get_x();      // 归一化x坐标
        s(2 * i + 1) = features[i].get_y(); // 归一化y坐标
    }
    return s;
}

Eigen::VectorXd MPC_IBVS_Controller::pixelToNormalized(
    const std::vector<vpImagePoint>& corners) const {
    
    if (corners.size() < 4) {
        throw std::invalid_argument("Need at least 4 corner points");
    }
    
    Eigen::VectorXd s(8);
    for (size_t i = 0; i < 4; i++) {
        double u_pixel = corners[i].get_u();
        double v_pixel = corners[i].get_v();
        
        double u_norm = (u_pixel - config_.cx) / config_.fx;
        double v_norm = (v_pixel - config_.cy) / config_.fy;
        
        s(2 * i) = u_norm;
        s(2 * i + 1) = v_norm;
    }
    
    return s;
}

std::vector<vpImagePoint> MPC_IBVS_Controller::normalizedToPixel(
    const Eigen::VectorXd& s) const {
    
    if (s.size() != 8) {
        throw std::invalid_argument("Feature vector must have size 8");
    }
    
    std::vector<vpImagePoint> corners(4);
    
    for (int i = 0; i < 4; i++) {
        double u_norm = s(2 * i);
        double v_norm = s(2 * i + 1);
        
        double u_pixel = u_norm * config_.fx + config_.cx;
        double v_pixel = v_norm * config_.fy + config_.cy;
        
        corners[i].set_u(u_pixel);
        corners[i].set_v(v_pixel);
    }
    
    return corners;
}

void MPC_IBVS_Controller::getFOVBounds(double& u_min, double& u_max, 
                                      double& v_min, double& v_max) const {
    u_min = u_min_;
    u_max = u_max_;
    v_min = v_min_;
    v_max = v_max_;
}

void MPC_IBVS_Controller::computeFOVBounds() {
    u_min_ = -config_.cx / config_.fx;
    u_max_ = (config_.image_width - config_.cx) / config_.fx;
    v_min_ = -config_.cy / config_.fy;
    v_max_ = (config_.image_height - config_.cy) / config_.fy;
}

Eigen::VectorXd MPC_IBVS_Controller::solveMPC(
    const std::vector<vpFeaturePoint>& p_current,
    const std::vector<vpFeaturePoint>& pd_target,
    const Eigen::VectorXd& tau_prev) {
    
    if (p_current.size() != 4 || pd_target.size() != 4) {
        ROS_ERROR("Need exactly 4 current and 4 target features");
        throw std::invalid_argument("Need exactly 4 current and 4 target features");
    }
    
    // 从ViSP特征中提取状态向量
    Eigen::VectorXd s_current = featuresToState(p_current);
    Eigen::VectorXd s_target = featuresToState(pd_target);
    
    //更新模型（使用p_current和pd_target）
    updateModel(p_current, pd_target);
    //重建预测矩阵
    buildPredictionMatrices();
    
    // 关键修改1：确保求解器对象已创建
    if (!solver_initialized_) {
        ROS_INFO("Creating OSQP solver object...");
        initializeSolver();  // 必须先创建求解器对象
    }
    
    // 设置并求解QP问题
    if (!setupQPProblem(s_current, s_target, tau_prev)) {
        ROS_ERROR("Failed to setup QP problem");
         return Eigen::VectorXd::Constant(6, 0.0);  // 返回-1向量表示错误
    }
    
    // 清理旧的求解器
    solver_->clearSolver();  
    //调用库函数initSolver()进行初始化
    if (!solver_->initSolver()) {
        ROS_ERROR("Failed to init OSQP solver");
         return Eigen::VectorXd::Constant(6, 0.0);  // 返回-1向量表示错误
    }
    //调用求解
    if (solver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        ROS_ERROR("Failed to solve QP problem");
         return Eigen::VectorXd::Constant(6, 0.0);  // 返回-1向量表示错误
    }
    
    // 提取解
    Eigen::VectorXd solution = solver_->getSolution();
    int n_ctrl = 6;
    
    ROS_DEBUG("MPC solved successfully");
    return solution.head(n_ctrl);
}


void MPC_IBVS_Controller::initializeSolver() {
    //完整的求解器初始化
    try {
        // 先清除已存在的求解器
        solver_.reset();
        // 创建新的求解器实例
        solver_ = std::make_unique<OsqpEigen::Solver>();
        // 设置求解器参数
        solver_->settings()->setWarmStart(true);
        solver_->settings()->setVerbosity(false);
        solver_->settings()->setAbsoluteTolerance(1e-6);
        solver_->settings()->setRelativeTolerance(1e-6);
        solver_->settings()->setMaxIteration(4000);
        
        // 标记为已初始化
        solver_initialized_ = true;
        
        ROS_DEBUG("OSQP solver initialized successfully");
        
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize OSQP solver: %s", e.what());
        solver_initialized_ = false;
        throw;
    }
}

bool MPC_IBVS_Controller::setupQPProblem(
    const Eigen::VectorXd& s_current,
    const Eigen::VectorXd& s_target,
    const Eigen::VectorXd& tau_prev) {

    int Np     = config_.Np;
    int n_ctrl = 6;
    int n_var  = n_ctrl * Np;

    /* 1. 构建 QP 目标函数 (H, f) */
    Eigen::SparseMatrix<double> H;
    Eigen::VectorXd f;
    computeQPMatrices(s_current, s_target, H, f);

    /* 2. 计算约束数量 
       控制约束: n_ctrl * Np
       视野约束: 每步 4 个特征点，每个点有 u, v 两个维度 = 8 * Np
    */
    int n_ctrl_con = n_ctrl * Np;
    int n_fov_con  = 8 * Np; 
    int n_con      = n_ctrl_con + n_fov_con;
    
    // 初始化 A, l, u，使用大数值填充 l 和 u 以防脏数据
    Eigen::VectorXd l_con = Eigen::VectorXd::Constant(n_con, -1e10);
    Eigen::VectorXd u_con = Eigen::VectorXd::Constant(n_con, 1e10);
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(n_con * n_ctrl); // 预留空间

    int offset = 0;

    /* 3. 构建控制约束 (速度限制) */
    buildControlConstraints(triplets, l_con, u_con, offset);

    /* 4. 构建视野约束 (FOV) */
    buildFOVConstraints(s_current, triplets, l_con, u_con, offset);

    /* 5. 组装稀疏矩阵 */
    Eigen::SparseMatrix<double> A_con(n_con, n_var);
    A_con.setFromTriplets(triplets.begin(), triplets.end());
    A_con.makeCompressed();

    /* 6. 数值安全性检查 (解决 4.6655e-310 问题) */
    for (int i = 0; i < n_con; ++i) {
        if (l_con(i) > u_con(i)) {
            // 物理意义上 l 稍微大于 u 通常是因为浮点误差，强制拉平
            double mid = (l_con(i) + u_con(i)) / 2.0;
            l_con(i) = mid - 1e-9;
            u_con(i) = mid + 1e-9;
        }
    }

    /* 7. 配置求解器数据 */
    if (!solver_) return false;
    
    // 必须清除旧数据，否则维度不匹配会崩溃
    solver_->data()->clearHessianMatrix();
    solver_->data()->clearLinearConstraintsMatrix();

    solver_->data()->setNumberOfVariables(n_var);
    solver_->data()->setNumberOfConstraints(n_con);
    solver_->data()->setHessianMatrix(H);
    solver_->data()->setGradient(f);
    solver_->data()->setLinearConstraintsMatrix(A_con);
    solver_->data()->setLowerBound(l_con);
    solver_->data()->setUpperBound(u_con);

    // 设置初始猜测
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n_var);
    for (int i = 0; i < Np; i++) x0.segment(i * n_ctrl, n_ctrl) = tau_prev;
    solver_->setPrimalVariable(x0);

    return true;
}


/*
bool MPC_IBVS_Controller::setupQPProblem(
    const Eigen::VectorXd& s_current,
    const Eigen::VectorXd& s_target,
    const Eigen::VectorXd& tau_prev) {

    int Np     = config_.Np;
    int n_ctrl = 6;
    int n_var  = n_ctrl * Np;

    //构建QP目标
    Eigen::SparseMatrix<double> H;
    Eigen::VectorXd f;
    computeQPMatrices(s_current, s_target, H, f);

    //约束数量
    int n_ctrl_con = n_ctrl * Np;     //速度约束
    int n_fov_con  = 16 * Np;         //视野约束
    int n_con      = n_ctrl_con + n_fov_con;
    
    // 调试：打印约束数量
    ROS_INFO_STREAM("Total constraints: " << n_con 
                    << " (ctrl: " << n_ctrl_con 
                    << ", fov: " << n_fov_con << ")");

    //创建 A, l, u 
    Eigen::SparseMatrix<double> A_con(n_con, n_var);
    Eigen::VectorXd l_con = Eigen::VectorXd::Zero(n_con);
    Eigen::VectorXd u_con = Eigen::VectorXd::Zero(n_con);
    
    // 调试：检查初始化后的边界值
    for (int i = 0; i < std::min(10, n_con); ++i) {
        ROS_DEBUG_STREAM("Initial bound[" << i << "]: l=" << l_con(i) 
                        << ", u=" << u_con(i));
    }

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(n_ctrl_con + n_fov_con * 10);

    int offset = 0;

    //控制约束
    buildControlConstraints(triplets, l_con, u_con, offset);

    //视野约束
    buildFOVConstraints(s_current, triplets, l_con, u_con, offset);

    //构建A_con
    A_con.setFromTriplets(triplets.begin(), triplets.end());
    A_con.makeCompressed();
    
    // 关键调试：检查所有边界值
    ROS_INFO("Checking constraint bounds...");
    for (int i = 0; i < n_con; ++i) {
        if (std::isnan(l_con(i)) || std::isnan(u_con(i))) {
            ROS_ERROR("NaN detected at bound[%d]: l=%f, u=%f", 
                     i, l_con(i), u_con(i));
        }
        if (std::isinf(l_con(i)) || std::isinf(u_con(i))) {
            ROS_ERROR("Inf detected at bound[%d]: l=%f, u=%f", 
                     i, l_con(i), u_con(i));
        }
        if (l_con(i) > u_con(i)) {
            ROS_ERROR("Invalid bound at [%d]: l=%f > u=%f", 
                     i, l_con(i), u_con(i));
        }
    }

    //初始猜测
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n_var);
    for (int i = 0; i < Np; i++) {
        x0.segment(i * n_ctrl, n_ctrl) = tau_prev;
    }
    
    // 关键修改：检查求解器是否存在
    if (!solver_) {
        ROS_ERROR("Solver is nullptr in setupQPProblem");
        return false;
    }
    
    //清除之前的旧数据
    solver_->data()->clearHessianMatrix();
    solver_->data()->clearLinearConstraintsMatrix();

    //设置OSQP
    solver_->data()->setNumberOfVariables(n_var);
    solver_->data()->setNumberOfConstraints(n_con);

    solver_->data()->setHessianMatrix(H);
    solver_->data()->setGradient(f);
    solver_->data()->setLinearConstraintsMatrix(A_con);
    solver_->data()->setLowerBound(l_con);
    solver_->data()->setUpperBound(u_con);

    solver_->setPrimalVariable(x0);

    solver_->settings()->setWarmStart(true);
    solver_->settings()->setVerbosity(false);

    ROS_DEBUG_STREAM("QP setup done. Vars: " << n_var
                     << ", Cons: " << n_con
                     << ", nnz(A): " << A_con.nonZeros());

    return true;
}
*/

/*
bool MPC_IBVS_Controller::setupQPProblem(
    const Eigen::VectorXd& s_current,
    const Eigen::VectorXd& s_target,
    const Eigen::VectorXd& tau_prev) {
    
    int Np = config_.Np;
    int n_ctrl = 6;
    int n_var = n_ctrl * Np;

    //构建 QP 目标
    Eigen::SparseMatrix<double> H;
    Eigen::VectorXd f;
    computeQPMatrices(s_current, s_target, H, f);

    //约束数量
    int n_ctrl_con = n_ctrl * Np;
    int n_fov_con  = 16 * Np;
    int n_con      = n_ctrl_con + n_fov_con;

    //约束边界
    Eigen::VectorXd l_con = Eigen::VectorXd::Zero(n_con);
    Eigen::VectorXd u_con = Eigen::VectorXd::Zero(n_con);


    //Triplets
    std::vector<Eigen::Triplet<double>> all_triplets;
    all_triplets.reserve(n_ctrl_con + n_fov_con * 10); 
    //*10 是保守估计，避免频繁扩容

    //控制约束
    int con_idx = 0;
    for (int k = 0; k < Np; k++) {
        for (int j = 0; j < n_ctrl; j++) {

            // A_con(row, col) = 1
            all_triplets.emplace_back(con_idx, k * n_ctrl + j, 1.0);

            if (j < 3) {
                // 线速度
                l_con(con_idx) = -config_.v_max;
                u_con(con_idx) =  config_.v_max;
            } else {
                // 角速度
                l_con(con_idx) = -config_.w_max;
                u_con(con_idx) =  config_.w_max;
            }

            con_idx++;
        }
    }

    //视野约束
    int fov_offset = n_ctrl_con;

    Eigen::SparseMatrix<double> A_fov_temp(n_fov_con, n_var);
    Eigen::VectorXd l_fov = Eigen::VectorXd::Zero(n_fov_con);
    Eigen::VectorXd u_fov = Eigen::VectorXd::Zero(n_fov_con);

    computeFOVConstraints(s_current, A_fov_temp, l_fov, u_fov);

    //提取 FOV triplets
    for (int k = 0; k < A_fov_temp.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A_fov_temp, k); it; ++it) {
            all_triplets.emplace_back(
                fov_offset + it.row(),
                it.col(),
                it.value()
            );
        }
    }

    //合并 FOV 边界
    for (int i = 0; i < n_fov_con; i++) {
        l_con(fov_offset + i) = l_fov(i);
        u_con(fov_offset + i) = u_fov(i);
    }

    //构建 A_con
    Eigen::SparseMatrix<double> A_con(n_con, n_var);
    A_con.setFromTriplets(all_triplets.begin(), all_triplets.end());
    A_con.makeCompressed();
    
    //设置初始猜测
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n_var);
    for (int i = 0; i < Np; i++) {
        x0.segment(i * n_ctrl, n_ctrl) = tau_prev;
    }
    
    // 调试信息
    ROS_DEBUG_STREAM("QP Problem dimensions: Variables: " << n_var 
                     << ", Constraints: " << n_con 
                     << ", Hessian nonzeros: " << H.nonZeros()
                     << ", Constraint matrix nonzeros: " << A_con.nonZeros());
    
    // 设置求解器
    solver_->data()->setNumberOfVariables(n_var);				//设置优化变量的个数
    solver_->data()->setNumberOfConstraints(n_con);				//约束个数
    
    if (!solver_->data()->setHessianMatrix(H)) {				//设置Hissen矩阵H
        ROS_ERROR("Failed to set Hessian matrix");
        return false;
    }
    
    if (!solver_->data()->setGradient(f)) {						//设置梯度向量f
        ROS_ERROR("Failed to set gradient");
        return false;
    }
    
    if (!solver_->data()->setLinearConstraintsMatrix(A_con)) {	//设置约束矩阵
        ROS_ERROR("Failed to set constraint matrix");
        return false;
    }
    
    if (!solver_->data()->setLowerBound(l_con)) {				//设置约束下界
        ROS_ERROR("Failed to set lower bound");
        return false;
    }
    
    if (!solver_->data()->setUpperBound(u_con)) {				//设置约束上界
        ROS_ERROR("Failed to set upper bound");
        return false;
    }
    
    // 设置初始猜测
    solver_->setPrimalVariable(x0);
    
    // 设置求解器参数
    solver_->settings()->setWarmStart(true);
    solver_->settings()->setVerbosity(false);
    solver_->settings()->setAbsoluteTolerance(1e-6);
    solver_->settings()->setRelativeTolerance(1e-6);
    solver_->settings()->setMaxIteration(4000);
    
    // 初始化求解器
    //if (!solver_->initSolver()) {
    //    ROS_ERROR("Failed to initialize OSQP solver");
    //    return false;
    //}
    
    ROS_DEBUG("QP problem setup successfully");
    return true;
}*/

//构建预测矩阵M_tau和M_s
void MPC_IBVS_Controller::buildPredictionMatrices() {
    int Np = config_.Np;
    int n_state = 8;
    int n_ctrl = 6;
    
    M_s_ = Eigen::MatrixXd::Zero(n_state * Np, n_state);
    M_tau_ = Eigen::MatrixXd::Zero(n_state * Np, n_ctrl * Np);
    
    std::vector<Eigen::MatrixXd> A_powers(Np + 1);
    A_powers[0] = Eigen::MatrixXd::Identity(n_state, n_state);
    for (int i = 1; i <= Np; i++) {
        A_powers[i] = A_ * A_powers[i-1];
    }
    
    for (int i = 1; i <= Np; i++) {
        M_s_.block((i-1) * n_state, 0, n_state, n_state) = A_powers[i];
        
        for (int j = 1; j <= i; j++) {
            M_tau_.block((i-1) * n_state, (j-1) * n_ctrl, n_state, n_ctrl) = 
                A_powers[i-j] * B_;
        }
    }
    
    ROS_DEBUG("Prediction matrices built");
    ROS_DEBUG_STREAM("B matrix norm: " << B_.norm());
}
//构建Q_blk
Eigen::MatrixXd MPC_IBVS_Controller::buildBlockDiagonalQ() const {
    int Np = config_.Np;
    int n_state = 8;
    
    Eigen::MatrixXd Q_blk = Eigen::MatrixXd::Zero(n_state * Np, n_state * Np);
    
    for (int i = 0; i < Np; i++) {
        Q_blk.block(i * n_state, i * n_state, n_state, n_state) = config_.Q;
    }
    
    return Q_blk;
}
//构建R_blk
Eigen::MatrixXd MPC_IBVS_Controller::buildBlockDiagonalR() const {
    int Np = config_.Np;
    int n_ctrl = 6;
    
    Eigen::MatrixXd R_blk = Eigen::MatrixXd::Zero(n_ctrl * Np, n_ctrl * Np);
    
    for (int i = 0; i < Np; i++) {
        R_blk.block(i * n_ctrl, i * n_ctrl, n_ctrl, n_ctrl) = config_.R;
    }
    
    return R_blk;
}
//构建S_target
Eigen::VectorXd MPC_IBVS_Controller::buildTargetStateVector(const Eigen::VectorXd& s_target) const {
    int Np = config_.Np;
    int n_state = 8;
    
    Eigen::VectorXd S_target = Eigen::VectorXd::Zero(n_state * Np);
    
    for (int i = 0; i < Np; i++) {
        S_target.segment(i * n_state, n_state) = s_target;
    }
    
    return S_target;
}

//构建H和f
void MPC_IBVS_Controller::computeQPMatrices(
    const Eigen::VectorXd& s_current,
    const Eigen::VectorXd& s_target,
    Eigen::SparseMatrix<double>& H,
    Eigen::VectorXd& f) const {
    
    int Np = config_.Np;
    int n_ctrl = 6;
    int n_var = n_ctrl * Np;
    
    Eigen::MatrixXd Q_blk = buildBlockDiagonalQ();
    Eigen::MatrixXd R_blk = buildBlockDiagonalR();
    Eigen::VectorXd S_target = buildTargetStateVector(s_target);
    
    Eigen::MatrixXd H_dense = 2.0 * (M_tau_.transpose() * Q_blk * M_tau_ + R_blk);
    Eigen::VectorXd f_dense = 2.0 * M_tau_.transpose() * Q_blk * (M_s_ * s_current - S_target);
    
    H = H_dense.sparseView();
    f = f_dense;
}
void MPC_IBVS_Controller::buildControlConstraints(
    std::vector<Eigen::Triplet<double>>& triplets,
    Eigen::VectorXd& l,
    Eigen::VectorXd& u,
    int& offset) {

    int Np = config_.Np;
    int n_ctrl = 6;

    for (int k = 0; k < Np; k++) {
        for (int j = 0; j < n_ctrl; j++) {
            int row = offset;
            int col = k * n_ctrl + j;

            triplets.emplace_back(row, col, 1.0);

            if (j < 3) { // 线速度
                l(row) = -config_.v_max;
                u(row) =  config_.v_max;
            } else {    // 角速度
                l(row) = -config_.w_max;
                u(row) =  config_.w_max;
            }
            offset++;
        }
    }
}
/*
void MPC_IBVS_Controller::buildControlConstraints(
    std::vector<Eigen::Triplet<double>>& triplets,
    Eigen::VectorXd& l,
    Eigen::VectorXd& u,
    int& offset){

    int Np     = config_.Np;
    int n_ctrl = 6;

    for (int k = 0; k < Np; k++) {
        for (int j = 0; j < n_ctrl; j++) {

            int row = offset;
            int col = k * n_ctrl + j;

            triplets.emplace_back(row, col, 1.0);

            if (j < 3) {
            	//线速度约束
                l(row) = -config_.v_max;
                u(row) =  config_.v_max;
            } else {
            	//角速度约束
                l(row) = -config_.w_max;
                u(row) =  config_.w_max;
            }

            offset++;
        }
    }
}
*/
/*
//计算控制约束 l_con和u_con
void MPC_IBVS_Controller::computeControlConstraints(
    Eigen::SparseMatrix<double>& A_con,
    Eigen::VectorXd& l_con,
    Eigen::VectorXd& u_con) const {
    
    int Np = config_.Np;
    int n_ctrl = 6;
    int n_var = n_ctrl * Np;
    
    int con_idx = 0;
    
    for (int k = 0; k < Np; k++) {
        for (int i = 0; i < n_ctrl; i++) {
            A_con.coeffRef(con_idx, k * n_ctrl + i) = 1.0;
            if (i < 3) {
                l_con(con_idx) = -OSQP_INFTY;
                u_con(con_idx) = config_.v_max;
            } else {
                l_con(con_idx) = -OSQP_INFTY;
                u_con(con_idx) = config_.w_max;
            }
            con_idx++;
            
            A_con.coeffRef(con_idx, k * n_ctrl + i) = -1.0;
            if (i < 3) {
                l_con(con_idx) = -OSQP_INFTY;
                u_con(con_idx) = config_.v_max;
            } else {
                l_con(con_idx) = -OSQP_INFTY;
                u_con(con_idx) = config_.w_max;
            }
            con_idx++;
        }
    }
}
*/
/*
//计算视野约束l_con和u_con
void MPC_IBVS_Controller::computeFOVConstraints(
    const Eigen::VectorXd& s_current,
    Eigen::SparseMatrix<double>& A_con,
    Eigen::VectorXd& l_con,
    Eigen::VectorXd& u_con) const {
    
    int Np = config_.Np;
    int n_state = 8;
    int n_ctrl = 6;
    int n_var = n_ctrl * Np;
    
    //C_total提取每个特征点的特定坐标用于约束
    Eigen::MatrixXd C_total = Eigen::MatrixXd::Zero(16 * Np, n_state * Np);
    
    for (int k = 0; k < Np; k++) {
        Eigen::MatrixXd C_k = Eigen::MatrixXd::Zero(16, n_state);
        
        for (int i = 0; i < 4; i++) {
            C_k(4*i, 2*i) = 1.0;			//u_i
            C_k(4*i + 1, 2*i) = -1.0;		//-u_i
            C_k(4*i + 2, 2*i + 1) = 1.0;	//v_i
            C_k(4*i + 3, 2*i + 1) = -1.0;	//-v_i
        }
        //构造C_total
        C_total.block(16*k, n_state*k, 16, n_state) = C_k;
    }
    
    //视野约束上界b
    Eigen::VectorXd b = Eigen::VectorXd::Zero(16 * Np);
    for (int k = 0; k < Np; k++) {
        for (int i = 0; i < 4; i++) {
            b(16*k + 4*i) = u_max_;
            b(16*k + 4*i + 1) = -u_min_;
            b(16*k + 4*i + 2) = v_max_;
            b(16*k + 4*i + 3) = -v_min_;
        }
    }
    //A_fov
    Eigen::MatrixXd A_fov_dense = C_total * M_tau_;
    //b_fov
    Eigen::VectorXd b_fov = b - C_total * M_s_ * s_current;
    
    for (int i = 0; i < A_fov_dense.rows(); i++) {
        for (int j = 0; j < A_fov_dense.cols(); j++) {
            if (std::abs(A_fov_dense(i, j)) > 1e-6) {
                A_con.coeffRef(i, j) = A_fov_dense(i, j);
            }
        }
    }
    
    l_con = Eigen::VectorXd::Constant(16 * Np, -OSQP_INFTY);
    u_con = b_fov;
}
*/
void MPC_IBVS_Controller::buildFOVConstraints(
    const Eigen::VectorXd& s_current,
    std::vector<Eigen::Triplet<double>>& triplets,
    Eigen::VectorXd& l,
    Eigen::VectorXd& u,
    int& offset) {

    int Np = config_.Np;
    int n_state = 8;
    int n_ctrl = 6;
    int n_features = 4;
    
    // 计算归一化平面下的图像边界
    double u_min = -config_.cx / config_.fx;
    double u_max = (config_.image_width - config_.cx) / config_.fx;
    double v_min = -config_.cy / config_.fy;
    double v_max = (config_.image_height - config_.cy) / config_.fy;
    
    // 遍历预测步 Np
    for (int j = 0; j < Np; j++) {
        // 当前预测步 j 下，s_pred = A^(j+1)*s_current + sum_{i=0}^{j} A^(j-i)*B*tau_i
        // 我们需要把变量 tau 移到 A 矩阵中，常数项 A^(j+1)*s_current 移到边界 l, u 中
        
        Eigen::MatrixXd A_pow_next = Eigen::MatrixXd::Identity(n_state, n_state);
        for(int k=0; k<=j; ++k) A_pow_next = A_ * A_pow_next;
        Eigen::VectorXd s_bias = A_pow_next * s_current;

        for (int feat = 0; feat < n_features; feat++) {
            // 处理 U 坐标
            int row_u = offset++;
            // 处理 V 坐标
            int row_v = offset++;

            // 填充 A 矩阵中的系数 (即 M_tau 的对应行)
            for (int i = 0; i <= j; i++) {
                Eigen::MatrixXd A_pow_ji = Eigen::MatrixXd::Identity(n_state, n_state);
                for(int k=0; k < (j-i); ++k) A_pow_ji = A_ * A_pow_ji;
                
                Eigen::MatrixXd B_block = A_pow_ji * B_;
                
                // 对应控制步 i 的列起始索引
                int col_start = i * n_ctrl;

                // 填入 u 的系数 (2*feat 行)
                for(int m=0; m<n_ctrl; ++m) {
                    double val = B_block(2*feat, m);
                    if(std::abs(val) > 1e-9) triplets.emplace_back(row_u, col_start + m, val);
                }
                // 填入 v 的系数 (2*feat + 1 行)
                for(int m=0; m<n_ctrl; ++m) {
                    double val = B_block(2*feat+1, m);
                    if(std::abs(val) > 1e-9) triplets.emplace_back(row_v, col_start + m, val);
                }
            }

            // 更新边界：u_min <= u_bias + A_row*tau <= u_max
            l(row_u) = u_min - s_bias(2 * feat);
            u(row_u) = u_max - s_bias(2 * feat);
            
            l(row_v) = v_min - s_bias(2 * feat + 1);
            u(row_v) = v_max - s_bias(2 * feat + 1);
        }
    }
}
/*
void MPC_IBVS_Controller::buildFOVConstraints(
    const Eigen::VectorXd& s_current,			//当前特征点状态
    std::vector<Eigen::Triplet<double>>& triplets,	//构造稀疏矩阵A_con
    Eigen::VectorXd& l,				//下边界
    Eigen::VectorXd& u,				//上边界
    int& offset) {

    int Np = config_.Np;
    int n_state = 8;
    int n_ctrl = 6;
    int n_features = 4;
    
    //使用局部定义的无穷大，避免OSQP_INFTY可能未定义
    const double INF_LOCAL = 1e10;
    
    //计算FOV边界
    double u_min = -config_.cx / config_.fx;
    double u_max = (config_.image_width - config_.cx) / config_.fx;
    double v_min = -config_.cy / config_.fy;
    double v_max = (config_.image_height - config_.cy) / config_.fy;
    
    ROS_DEBUG_STREAM("FOV bounds: u_min=" << u_min << ", u_max=" << u_max 
                     << ", v_min=" << v_min << ", v_max=" << v_max);
    
    // 逐预测步构造约束（MATLAB风格）
    for (int j = 0; j < Np; j++) {
        // 计算当前步的预测状态（无控制输入）
        Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(n_state, n_state);
        for (int k = 0; k <= j; k++) {
            A_power = A_ * A_power;
        }
        Eigen::VectorXd s_pred = A_power * s_current;  // 预测的归一化坐标
        
        // 初始化系数矩阵
        Eigen::MatrixXd Phi_u = Eigen::MatrixXd::Zero(n_features, n_ctrl * Np);
        Eigen::MatrixXd Phi_v = Eigen::MatrixXd::Zero(n_features, n_ctrl * Np);
        
        // 构造Φ矩阵
        for (int i = 0; i <= j; i++) {
            // 计算A^(j-i)
            Eigen::MatrixXd A_power_ji = Eigen::MatrixXd::Identity(n_state, n_state);
            for (int k = 0; k < (j - i); k++) {
                A_power_ji = A_ * A_power_ji;
            }
            Eigen::MatrixXd B_block = A_power_ji * B_;
            
            int col_start = i * n_ctrl;
            
            // 提取u和v的系数
            for (int feat = 0; feat < n_features; feat++) {
                // u的系数（s中的奇数行：0,2,4,6）
                Phi_u.row(feat).segment(col_start, n_ctrl) = 
                    B_block.row(2 * feat);
                // v的系数（s中的偶数行：1,3,5,7）
                Phi_v.row(feat).segment(col_start, n_ctrl) = 
                    B_block.row(2 * feat + 1);
            }
        }
        
        // 构造当前步的约束
        for (int feat = 0; feat < n_features; feat++) {
            // 提取预测的归一化坐标
            double u_norm = s_pred(2 * feat);
            double v_norm = s_pred(2 * feat + 1);
            
            // 约束1: u >= u_min  => -Φ_u * τ ≤ u_min - u_norm
            for (int col = 0; col < n_ctrl * Np; col++) {
                double coeff = -Phi_u(feat, col);
                if (std::abs(coeff) > 1e-9) {
                    triplets.emplace_back(offset, col, coeff);
                }
            }
            l(offset) = -INF_LOCAL;
            u(offset) = u_min - u_norm;
            offset++;
            
            // 约束2: u <= u_max  => Φ_u * τ ≤ u_max - u_norm
            for (int col = 0; col < n_ctrl * Np; col++) {
                double coeff = Phi_u(feat, col);
                if (std::abs(coeff) > 1e-9) {
                    triplets.emplace_back(offset, col, coeff);
                }
            }
            l(offset) = -INF_LOCAL;
            u(offset) = u_max - u_norm;
            offset++;
            
            // 约束3: v >= v_min  => -Φ_v * τ ≤ v_min - v_norm
            for (int col = 0; col < n_ctrl * Np; col++) {
                double coeff = -Phi_v(feat, col);
                if (std::abs(coeff) > 1e-9) {
                    triplets.emplace_back(offset, col, coeff);
                }
            }
            l(offset) = -INF_LOCAL;
            u(offset) = v_min - v_norm;
            offset++;
            
            // 约束4: v <= v_max  => Φ_v * τ ≤ v_max - v_norm
            for (int col = 0; col < n_ctrl * Np; col++) {
                double coeff = Phi_v(feat, col);
                if (std::abs(coeff) > 1e-9) {
                    triplets.emplace_back(offset, col, coeff);
                }
            }
            l(offset) = -INF_LOCAL;
            u(offset) = v_max - v_norm;
            offset++;
        }
        
        // 调试信息
        ROS_DEBUG_STREAM("Step " << j << ": u_norm=[" 
                         << s_pred(0) << "," << s_pred(2) << "," << s_pred(4) << "," << s_pred(6) << "]"
                         << ", v_norm=[" 
                         << s_pred(1) << "," << s_pred(3) << "," << s_pred(5) << "," << s_pred(7) << "]");
    }
    
    ROS_DEBUG_STREAM("Built FOV constraints. Total rows: " << offset);
}
*/

/*
void MPC_IBVS_Controller::buildFOVConstraints(
    const Eigen::VectorXd& s_current,
    std::vector<Eigen::Triplet<double>>& triplets,
    Eigen::VectorXd& l,
    Eigen::VectorXd& u,
    int& offset){

    int Np      = config_.Np;
    int n_state = 8;
    int n_ctrl  = 6;
    int n_var   = n_ctrl * Np;

    //构造 C_total
    Eigen::MatrixXd C_total = Eigen::MatrixXd::Zero(16 * Np, n_state * Np);

    for (int k = 0; k < Np; k++) {
        Eigen::MatrixXd C_k = Eigen::MatrixXd::Zero(16, n_state);

        for (int i = 0; i < 4; i++) {
            C_k(4*i,     2*i)     =  1.0;  //  u_i
            C_k(4*i + 1, 2*i)     = -1.0;  // -u_i
            C_k(4*i + 2, 2*i + 1) =  1.0;  //  v_i
            C_k(4*i + 3, 2*i + 1) = -1.0;  // -v_i
        }

        C_total.block(16*k, n_state*k, 16, n_state) = C_k;
    }

    //上界 b 
    Eigen::VectorXd b = Eigen::VectorXd::Zero(16 * Np);
    for (int k = 0; k < Np; k++) {
        for (int i = 0; i < 4; i++) {
            b(16*k + 4*i)     =  u_max_;
            b(16*k + 4*i + 1) = -u_min_;
            b(16*k + 4*i + 2) =  v_max_;
            b(16*k + 4*i + 3) = -v_min_;
        }
    }

    //映射到 τ
    Eigen::MatrixXd A_fov = C_total * M_tau_;
    Eigen::VectorXd b_fov = b - C_total * M_s_ * s_current;

    //写入总约束
    for (int i = 0; i < A_fov.rows(); i++) {
        for (int j = 0; j < A_fov.cols(); j++) {
            if (std::abs(A_fov(i, j)) > 1e-9) {
                triplets.emplace_back(offset + i, j, A_fov(i, j));
            }
        }
        l(offset + i) = -OSQP_INFTY;
        u(offset + i) = b_fov(i);
    }

    offset += A_fov.rows();
}*/
