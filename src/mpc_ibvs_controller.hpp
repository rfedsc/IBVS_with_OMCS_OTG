#define MPC_IBVS_CONTROLLER_HPP
#define MPC_IBVS_CONTROLLER_HPP

#include <Eigen/Dense>					//线性代数核心库
#include <vector>						//
#include <memory>
#include <visp3/core/vpImagePoint.h>	//ViSP的像素点类型
//OSQP求解器
#include <QsqpEigen/OsqpEigen.h>

struct MPCConfig{
	int Np = 6;						//预测时域
	double Ts = 0.04;				//采样时间
	double z_star = 0.1;			//期望深度
	double v_max = 0.5;				//最大线速度
	double w_max = 0.5;				//最大角速度
	double image_width = 640.0;		//图像宽度
	double image_height = 480.0;	//图像高度

	//权重矩阵
	Eigen::Matrix<double,8,8> Q;
	Eigen::Matrix<double,6,6> R;
	double W_weight = 8.0;
	
	//相机内参
	double fx = 600.75;
	double fy = 601.0;
	double cx = 325.882;
	double cy = 250.568;
	
	MPCConfig(){
		Q = Eigen::Matrix<double,8,8>::Identity()*10.0;
		R = Eigen::Matrix<double,6,6>::Identity()*1.0;
	}
};

class MPC_IBVS_Controller{
private:
	MPCConfig config_;
	Eigen::Matrix<double,8,8> A_;					//状态矩阵
	Eigen::Matrix<double,8,6> B_;					//控制矩阵
	
	std::unique_ptr<QsqpEigen::Solver> solver_;		//QP求解器
	bool solver_initialized_;
	
	//视野边界约束
	double u_min_,u_max_,vmin_,v_max_;
	
public:
	MPC_IBVS_Controller(const MPCConfig& config = MPCConfig());
	//设置配置
	void setConfig(const MPCConfig& config);
	
	//计算交互矩阵
	Eigen::Matrix<double,8,6> calculateInteractionMatrix(const Eigen::VectorX& s, double Z) const;
	
	//更新线性化模型
	void updateModel(const Eigen::VectorXd& s);
	
	//像素坐标到归一化坐标
	Eigen::VectorXd pixelToNormalized(const std::vector<vpImagePoint>& corners) const;
	//归一化坐标到像素坐标
	std::vector<vpImagePoint> normalizedToPixel(const Eigen::VectorXd& s) const;
	
	//获取视野约束边界
	void getFOVBounds(double& u_min, double& u_max, double& v_main, double& v_max) const;
	//MPC优化求解
	Eigen::VectorXd solveMPC(const Eigen::VectorXd& s_current,
							 const Eigen::VectorXd& s_target,
							 const Eigen::VectorXd& tau_prev);
	//获取配置
	const MPCConfig& getConfig() const {return config_;}
	
private:
	//初始化求解器
	void initializeSolver();
	//计算视野约束边界
	void computeFOVBounds();
	//构建QP问题
	bool setupQPProblem(const Eigen::VectorXd& s_curreent,
						const Eigen::VectorXd& s_target,
						const Eigen::Veectord& tau_prev);
	//预测状态轨迹
	std::vector<Eigen::VectorXd> predictStateTrajectory(const Eigen::VectorXd& s_current,
														const Eigen::VectorXd& control_sequence) const;
	//计算视野约束
	void addFOVConstraints(Eigen::SparseMatrix<double>& A_con,
						   Eigen::VectorXd& l_con,
						   Eigen::VectorXd& u_con,
						   int& con_idx,
						   const Eigen::VectorXd& s_current,
						   const Eigen::VectorXd& control_sequence) const;
	
	//计算控制约束
	void addControlConstraints(Eigen::SparseMatrix<double>& A_con,
							   Eigen::VectorXd& l_con,
							   Eigen::VectorXd& u_con,
							   int& con_idx) const;
							   
};

#endif
































