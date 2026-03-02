#include "mpc_ibvs_controller.hpp"
#include <iostream>
#include <cmath>

//构造MPC_IBVS控制器对象,保存配置参数,初始化线性模型
//并预先计算图像视野约束边界
MPC_IBVS_Controller::MPC_IBVS_Controller(const MPCConfig& config)
	:config_(config),solver_initialized_(false){
	
	//初始化模型矩阵
	A_ = Eigen::Matrix<double,8,8>::Identity();
	B_ = Eigen::Matrix<double,8,6>::Zero();
	
	//计算视野边界
	computeFOVBounds();	
}

//在运行过程中,更新MPC参数,并强制让QP求解器重新初始化
void MPC_IBVS_Controller::setConfig(const MPCConfig& config){
	config_ = config;
	computeFOVBounds();
	solver_initialized_ = false;	//需要重新初始化求解器
}

//计算交互矩阵(基于归一化坐标)
Eigen::Matrix<double,8,6> MPC_IBVS_Controller::calculateInteractionMatrix(
	const Eigen::VectorXd& s,double Z) const{
	
	Eigen::Matrix<double,8,6> Ls = Eigen::Matrix<double,8,6>::Zero();
	for(int i=0;i<4;i++){
		double u = s(2*i);
		double v = s(2*i+1);
		double inv_Z = 1.0/Z;
		
		Ls.block<2,6>(2*i,0)<<
			-inv_Z,0,u*inv_Z,u*v,-(1+u*u),v,
			0,-inv_Z,v*inv_Z,1+v*v,-u*v,-u;
	}
	return Ls;
}

//计算控制矩阵B，更新线性化模型
void MPC_IBVS_Controller::updateModel(const Eigen::VectorXd& s){
	Eigen::Matrix<double,8,6> Ls = calculateInteractionMatrix(s,config_.z_star); //计算交互矩阵B
	B_ = config_.Ts*Ls;
}

//把像素坐标转化成归一化坐标
Eigen::VectorXd MPC_IBVS_Controller::pixelToNormalized(
	const std::vector<vpImagePoint>& corners) const{
	
	Eigen::VectorXd s(8);
	
	for(size_t i=0;i<4 && i<corners.size();i++){
		double u_pixel = corners[i].get_u();
		double v_pixel = corners[i].get_v();
		
		double u_norm = (u_pixel-config_.cx)/config_.fx;
		double v_norm = (v_pixel-config_.cy)/config_.fy;
		
		s(2*i) = u_norm;
		s(2*i+1) = v_norm;
	}
	
	return s;
}

//归一化坐标到像素坐标	
std::vector<vpImagePoint> MPC_IBVS_Controller::normalizedToPixel(
    const Eigen::VectorXd& s) const {
    
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

//获取视野边界
void MPC_IBVS_Controller::getFOVBounds(double& u_min, double& u_max, 
                                      double& v_min, double& v_max) const {
    u_min = u_min_;
    u_max = u_max_;
    v_min = v_min_;
    v_max = v_max_;
}

//把像素坐标下的图像边界转化为归一化坐标视野约束边界
void MPC_IBVS_Controller::computeFOVBounds() {
	//图像左边界
    u_min_ = -config_.cx / config_.fx;
    //图像右边界
    u_max_ = (config_.image_width - config_.cx) / config_.fx;
    //图像上边界
    v_min_ = -config_.cy / config_.fy;
    //图像下边界
    v_max_ = (config_.image_height - config_.cy) / config_.fy;
}


//根据当前的视觉特征,计算出当前这一时刻要发送的相机速度
Eigen::VectorXd MPC_IBVS_Controller::solveMPC(
    const Eigen::VectorXd& s_current, //当前视觉特征
    const Eigen::VectorXd& s_target,  //目标位置
    const Eigen::VectorXd& tau_prev) {
    
    //更新线性化模型
    updateModel(s_current);
    
    //初始化QP求解器,只初始化一次
    if (!solver_initialized_) {
        initializeSolver();
    }
    
    //构建QP问题
    if (!setupQPProblem(s_current, s_target, tau_prev)) {
        std::cerr << "Failed to setup QP problem" << std::endl;
        //如果失败,返回上一帧速度
        return tau_prev;
    }
    
    //调用QP求解器
    if (solver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "Failed to solve QP problem" << std::endl;
        //如果失败,返回上一帧速度
        return tau_prev;
    }
    
    //提取解向量
    Eigen::VectorXd solution = solver_->getSolution();
    int n_ctrl = 6;
    
    //只提取第一个控制向量
    return solution.head(n_ctrl);
}

//创建一个QP求解器对象
void MPC_IBVS_Controller::initializeSolver() {
    solver_ = std::make_unique<OsqpEigen::Solver>();
    //标记求解器已经初始化
    solver_initialized_ = true;
}

//把MPC问题写成OSQP能解的QP形式,并初始化求解器
bool MPC_IBVS_Controller::setupQPProblem(
    const Eigen::VectorXd& s_current,	//当前视觉特征
    const Eigen::VectorXd& s_target,	//目标特征
    const Eigen::VectorXd& tau_prev) {
    
    int Np = config_.Np;		//预测步数
    int n_state = 8;			//状态维数
    int n_ctrl = 6;				//控制维数
    int n_var = n_ctrl * Np;	//QP变量总数
    
    //构建预测时域的整个控制序列
    Eigen::VectorXd tau_seq0 = Eigen::VectorXd::Zero(n_var);
    //用上一个时刻的控制量作为未来所有预测步的初始猜测
    for (int i = 0; i < Np; i++) {
        tau_seq0.segment(i * n_ctrl, n_ctrl) = tau_prev;
    }
    
    // 构建Hessian矩阵P
    Eigen::SparseMatrix<double> P(n_var, n_var);
    // 控制成本：R权重
    for (int k = 0; k < Np; k++) {
        for (int i = 0; i < n_ctrl; i++) {
            P.insert(k * n_ctrl + i, k * n_ctrl + i) = config_.R(i, i);
        }
    }
    
    // 构建梯度向量q
    Eigen::VectorXd q = Eigen::VectorXd::Zero(n_var);
    
    // 计算预测轨迹的状态误差成本
    // 这里简化处理，实际应该在整个预测时域上计算
    std::vector<Eigen::VectorXd> s_pred = predictStateTrajectory(s_current, tau_seq0);
    
    // 约束数量
    int n_ctrl_con = 6 * Np * 2;  // 每个控制维度有上下界
    int n_fov_con = 4 * 4 * Np;   // 4个点×4个边界×Np步
    int n_con = n_ctrl_con + n_fov_con;
    
    // 约束矩阵
    Eigen::SparseMatrix<double> A_con(n_con, n_var);
    Eigen::VectorXd l_con = Eigen::VectorXd::Zero(n_con);
    Eigen::VectorXd u_con = Eigen::VectorXd::Zero(n_con);
    
    int con_idx = 0;
    
    // 添加控制约束
    addControlConstraints(A_con, l_con, u_con, con_idx);
    
    // 添加视野约束
    addFOVConstraints(A_con, l_con, u_con, con_idx, s_current, tau_seq0);
    
    // 设置求解器
    solver_->data()->setNumberOfVariables(n_var);
    solver_->data()->setNumberOfConstraints(n_con);
    solver_->data()->setHessianMatrix(P);
    solver_->data()->setGradient(q);
    solver_->data()->setLinearConstraintsMatrix(A_con);
    solver_->data()->setLowerBound(l_con);
    solver_->data()->setUpperBound(u_con);
    
    // 设置求解器参数
    solver_->settings()->setWarmStart(true);
    solver_->settings()->setVerbosity(false);
    solver_->settings()->setAbsoluteTolerance(1e-6);
    solver_->settings()->setRelativeTolerance(1e-6);
    solver_->settings()->setMaxIteration(4000);
    
    // 初始化求解器
    return solver_->initSolver();
}

std::vector<Eigen::VectorXd> MPC_IBVS_Controller::predictStateTrajectory(
    const Eigen::VectorXd& s_current,
    const Eigen::VectorXd& control_sequence) const {
    
    int Np = config_.Np;
    std::vector<Eigen::VectorXd> s_pred(Np + 1);
    s_pred[0] = s_current;
    
    for (int k = 0; k < Np; k++) {
        Eigen::VectorXd tau = control_sequence.segment(k * 6, 6);
        s_pred[k + 1] = A_ * s_pred[k] + B_ * tau;
    }
    
    return s_pred;
}

void MPC_IBVS_Controller::addControlConstraints(
    Eigen::SparseMatrix<double>& A_con,
    Eigen::VectorXd& l_con,
    Eigen::VectorXd& u_con,
    int& con_idx) const {
    
    int Np = config_.Np;
    
    for (int k = 0; k < Np; k++) {
        // 线速度约束
        for (int i = 0; i < 3; i++) {
            // 上界约束：v_i <= v_max
            A_con.insert(con_idx, k * 6 + i) = 1.0;
            l_con(con_idx) = -OsqpEigen::INFTY;
            u_con(con_idx) = config_.v_max;
            con_idx++;
            
            // 下界约束：-v_i <= v_max  => v_i >= -v_max
            A_con.insert(con_idx, k * 6 + i) = -1.0;
            l_con(con_idx) = -OsqpEigen::INFTY;
            u_con(con_idx) = config_.v_max;
            con_idx++;
        }
        
        // 角速度约束
        for (int i = 3; i < 6; i++) {
            // 上界约束
            A_con.insert(con_idx, k * 6 + i) = 1.0;
            l_con(con_idx) = -OsqpEigen::INFTY;
            u_con(con_idx) = config_.w_max;
            con_idx++;
            
            // 下界约束
            A_con.insert(con_idx, k * 6 + i) = -1.0;
            l_con(con_idx) = -OsqpEigen::INFTY;
            u_con(con_idx) = config_.w_max;
            con_idx++;
        }
    }
}

void MPC_IBVS_Controller::addFOVConstraints(
    Eigen::SparseMatrix<double>& A_con,
    Eigen::VectorXd& l_con,
    Eigen::VectorXd& u_con,
    int& con_idx,
    const Eigen::VectorXd& s_current,
    const Eigen::VectorXd& control_sequence) const {
    
    int Np = config_.Np;
    std::vector<Eigen::VectorXd> s_pred = predictStateTrajectory(s_current, control_sequence);
    
    for (int k = 1; k <= Np; k++) {  // 从第一步预测开始
        const Eigen::VectorXd& s_k = s_pred[k];
        
        for (int i = 0; i < 4; i++) {
            int u_idx = 2 * i;
            int v_idx = 2 * i + 1;
            
            // u_min <= u <= u_max
            // 转换为：-u <= -u_min 和 u <= u_max
            
            // 下界约束：u >= u_min
            A_con.insert(con_idx, 0) = 0;  // 简化处理，实际应与控制输入相关
            l_con(con_idx) = -OsqpEigen::INFTY;
            u_con(con_idx) = -u_min_;  // 注意负号
            con_idx++;
            
            // 上界约束：u <= u_max
            A_con.insert(con_idx, 0) = 0;
            l_con(con_idx) = -OsqpEigen::INFTY;
            u_con(con_idx) = u_max_;
            con_idx++;
            
            // v_min <= v <= v_max
            // 下界约束：v >= v_min
            A_con.insert(con_idx, 0) = 0;
            l_con(con_idx) = -OsqpEigen::INFTY;
            u_con(con_idx) = -v_min_;  // 注意负号
            con_idx++;
            
            // 上界约束：v <= v_max
            A_con.insert(con_idx, 0) = 0;
            l_con(con_idx) = -OsqpEigen::INFTY;
            u_con(con_idx) = v_max_;
            con_idx++;
        }
    }
}



























