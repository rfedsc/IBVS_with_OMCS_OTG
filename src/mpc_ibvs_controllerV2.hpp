#ifndef MPC_IBVS_CONTROLLER_HPP
#define MPC_IBVS_CONTROLLER_HPP
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>

struct MPCConfig {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int Np = 6;
    double Ts = 0.033;
    double v_max = 100.0;
    double w_max = 100.0;
    double image_width = 640.0;
    double image_height = 480.0;
    
    vpServo::vpServoIteractionMatrixType interaction_type = vpServo::MEAN;
    
    Eigen::Matrix<double, 8, 8> Q;
    Eigen::Matrix<double, 6, 6> R;
    
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
    bool model_updated_;
    
    std::vector<vpFeaturePoint> p_current_;		//问题可能在这里
    std::vector<vpFeaturePoint> pd_target_;		//问题可能在这里
    
    mutable Eigen::MatrixXd M_s_;
    mutable Eigen::MatrixXd M_tau_;
    
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 修改这里：去掉默认参数
    MPC_IBVS_Controller(const MPCConfig& config);
    virtual ~MPC_IBVS_Controller();
    
    void setConfig(const MPCConfig& config);
    void updateModel(const std::vector<vpFeaturePoint>& p_current,
                     const std::vector<vpFeaturePoint>& pd_target);
    
    Eigen::VectorXd solveMPC(const std::vector<vpFeaturePoint>& p_current,
                            const std::vector<vpFeaturePoint>& pd_target,
                            const Eigen::VectorXd& tau_prev);
    
    const MPCConfig& getConfig() const { return config_; }
    const Eigen::Matrix<double, 8, 8>& getA() const { return A_; }
    const Eigen::Matrix<double, 8, 6>& getB() const { return B_; }
    
private:
    Eigen::VectorXd solveUnconstrainedQP(const Eigen::VectorXd& s_current,
                                        const Eigen::VectorXd& s_target);
    void buildPredictionMatrices();
    Eigen::MatrixXd buildBlockDiagonalQ() const;
    Eigen::MatrixXd buildBlockDiagonalR() const;
    Eigen::VectorXd buildTargetStateVector(const Eigen::VectorXd& s_target) const;
    Eigen::VectorXd featuresToState(const std::vector<vpFeaturePoint>& features) const;
    
    void computeQPMatricesDense(const Eigen::VectorXd& s_current,
                               const Eigen::VectorXd& s_target,
                               Eigen::MatrixXd& H,
                               Eigen::VectorXd& f) const;
};

#endif
