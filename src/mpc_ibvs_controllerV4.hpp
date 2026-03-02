 #ifndef MPC_IBVS_CONTROLLER_HPP
#define MPC_IBVS_CONTROLLER_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <memory>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <OsqpEigen/OsqpEigen.h>

#ifndef OSQP_INFTY
#define OSQP_INFTY 1e30
#endif

struct MPCConfig {
    int Np = 6;                     // 预测时域
    double Ts = 0.033;              // 采样时间
    double v_max = 0.5;             // 最大线速度
    double w_max = 0.5;             // 最大角速度
    double image_width = 640.0;     // 图像宽度
    double image_height = 480.0;    // 图像高度
    
    // 交互矩阵类型
    vpServo::vpServoIteractionMatrixType interaction_type = vpServo::MEAN;
    
    // 权重矩阵
    Eigen::Matrix<double, 8, 8> Q;
    Eigen::Matrix<double, 6, 6> R;
    
    // 相机内参
    double fx = 600.75;
    double fy = 601.0;
    double cx = 325.882;
    double cy = 250.568;
    
    MPCConfig() {
        Q = Eigen::Matrix<double, 8, 8>::Identity() * 1.0;
        R = Eigen::Matrix<double, 6, 6>::Identity() * 1.0;
    }
};

class MPC_IBVS_Controller {
private:
    MPCConfig config_;
    Eigen::Matrix<double, 8, 8> A_;
    Eigen::Matrix<double, 8, 6> B_;
    std::unique_ptr<OsqpEigen::Solver> solver_;
    bool solver_initialized_;
    bool model_updated_;
    
    // ViSP任务对象（用于计算交互矩阵）
    vpServo task_;
    std::vector<vpFeaturePoint> p_current_;  // 当前特征
    std::vector<vpFeaturePoint> pd_target_;  // 目标特征
    
    // 视野边界
    double u_min_, u_max_, v_min_, v_max_;
    
    // 预测矩阵
    mutable Eigen::MatrixXd M_s_;
    mutable Eigen::MatrixXd M_tau_;
    
public:
    MPC_IBVS_Controller(const MPCConfig& config = MPCConfig());
    
    // 设置配置
    void setConfig(const MPCConfig& config);
    
    // 使用ViSP特征更新线性化模型
    void updateModel(const std::vector<vpFeaturePoint>& p_current,
                     const std::vector<vpFeaturePoint>& pd_target);
    
    // 将像素坐标转换为归一化坐标
    Eigen::VectorXd pixelToNormalized(const std::vector<vpImagePoint>& corners) const;
    
    // 将归一化坐标转换为像素坐标
    std::vector<vpImagePoint> normalizedToPixel(const Eigen::VectorXd& s) const;
    
    // 获取视野边界
    void getFOVBounds(double& u_min, double& u_max, double& v_min, double& v_max) const;
    
    // MPC优化求解
    Eigen::VectorXd solveMPC(const std::vector<vpFeaturePoint>& p_current,
                            const std::vector<vpFeaturePoint>& pd_target,
                            const Eigen::VectorXd& tau_prev);
    
    // 获取配置
    const MPCConfig& getConfig() const { return config_; }
    
    // 获取当前模型矩阵
    const Eigen::Matrix<double, 8, 8>& getA() const { return A_; }
    const Eigen::Matrix<double, 8, 6>& getB() const { return B_; }
    
    void buildControlConstraints(std::vector<Eigen::Triplet<double>>& triplets,
    					      Eigen::VectorXd& l,
    					      Eigen::VectorXd& u,
    						  int& offset);
    						  
	void buildFOVConstraints(const Eigen::VectorXd& s_current,
    						  std::vector<Eigen::Triplet<double>>& triplets,
    						  Eigen::VectorXd& l,
    						  Eigen::VectorXd& u,
    						  int& offset);	
    
private:
    // 初始化求解器
    void initializeSolver();
    
    // 初始化ViSP任务
    void initializeVISPTask();
    
    // 计算视野边界
    void computeFOVBounds();
    
    // 设置QP问题
    bool setupQPProblem(const Eigen::VectorXd& s_current,
                       const Eigen::VectorXd& s_target,
                       const Eigen::VectorXd& tau_prev);
    
    // 构建预测矩阵
    void buildPredictionMatrices();
    
    // 构建块对角权重矩阵
    Eigen::MatrixXd buildBlockDiagonalQ() const;
    Eigen::MatrixXd buildBlockDiagonalR() const;
    
    // 计算QP矩阵H和f
    void computeQPMatrices(const Eigen::VectorXd& s_current,
                          const Eigen::VectorXd& s_target,
                          Eigen::SparseMatrix<double>& H,
                          Eigen::VectorXd& f) const;
    
    // 计算视野约束
    //void computeFOVConstraints(const Eigen::VectorXd& s_current,
    //                          Eigen::SparseMatrix<double>& A_con,
    //                          Eigen::VectorXd& l_con,
    //                          Eigen::VectorXd& u_con) const;
                                
    
    // 计算控制约束
    //void computeControlConstraints(Eigen::SparseMatrix<double>& A_con,
    //                              Eigen::VectorXd& l_con,
    //                              Eigen::VectorXd& u_con) const;
    
    // 构建目标状态向量
    Eigen::VectorXd buildTargetStateVector(const Eigen::VectorXd& s_target) const;
    
    // 从ViSP特征提取状态向量
    Eigen::VectorXd featuresToState(const std::vector<vpFeaturePoint>& features) const;
    
    // 从ViSP任务计算交互矩阵
    Eigen::Matrix<double, 8, 6> computeInteractionMatrixFromTask();
    
    // 将ViSP矩阵转换为Eigen矩阵
    Eigen::Matrix<double, 8, 6> convertVispToEigen(const vpMatrix& L_visp) const;
    
    // 计算单个点的交互矩阵（兼容旧方法）
    Eigen::Matrix<double, 8, 6> computeInteractionMatrixFromFeaturesOld(
        const std::vector<vpFeaturePoint>& p_current);
};

#endif // MPC_IBVS_CONTROLLER_HPP
