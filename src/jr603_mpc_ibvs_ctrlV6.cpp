#include "mpc_ibvs_controllerV4.hpp"
#include <ros/ros.h>
#include <iostream>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <cmath>
#include <iomanip>
#include <geometry_msgs/Point.h>
#include <lys_visp_demo/PixelCoordinates.h>

class JR603_MPC_IBVS {
private:
    ros::NodeHandle nh_;
    // 速度发布器
    ros::Publisher vel_pub_;
    // 期望位置发布器
    ros::Publisher pd_pub_;
    // 期望像素坐标发布器
    ros::Publisher pixel_pub_;
    
    // MPC控制器
    std::unique_ptr<MPC_IBVS_Controller> mpc_controller_;
    
    // ViSP相关
    vpCameraParameters cam_;
    vpImage<unsigned char> I_;
    vpDisplayX display_;
    vpDetectorAprilTag detector_;
    
    // 特征和点
    std::vector<vpFeaturePoint> p_, pd_;
    std::vector<vpPoint> points_;
    
    // 位姿矩阵
    vpHomogeneousMatrix eMc_;
    vpHomogeneousMatrix cdMo_;
    vpHomogeneousMatrix oMo_;
    
    // 配置参数
    double opt_tagSize_;
    bool display_tag_;
    int opt_quad_decimate_;
    bool opt_plot_;
    double convergence_threshold_;
    
    // 状态变量
    bool has_converged_;
    bool send_velocities_;
    bool first_time_;
    
    // 绘图
    vpPlot* plotter_;
    int iter_plot_;
    
    // 控制历史
    Eigen::VectorXd tau_prev_;
    bool pd_initialized_;
    
public:
    JR603_MPC_IBVS(ros::NodeHandle& nh) 
        : nh_(nh), 
          has_converged_(false),
          first_time_(true),
          iter_plot_(0),
          pd_initialized_(false) {
        
        // 初始化发布器
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/camera_velocity", 10);
        pd_pub_ = nh_.advertise<geometry_msgs::Point>("/pd_publish", 10);
        pixel_pub_ = nh_.advertise<lys_visp_demo::PixelCoordinates>("/image_pixel", 10);
        
        // 加载参数
        loadParameters();
        // 初始化ViSP
        initializeViSP();
        // 初始化可视化
        if (opt_plot_) {
            initializePlotter();
        }
        // 初始化控制输入
        tau_prev_ = Eigen::VectorXd::Zero(6);
        
        ROS_INFO("JR603 MPC-IBVS Controller initialized");
    }
    
    void loadParameters() {
        nh_.param("tag_size", opt_tagSize_, 0.0815);
        nh_.param("display_tag", display_tag_, true);
        nh_.param("quad_decimate", opt_quad_decimate_, 2);
        nh_.param("plot", opt_plot_, true);
        nh_.param("convergence_threshold", convergence_threshold_, 0.001);
        nh_.param("send_velocities", send_velocities_, true);
        
        ROS_INFO("Parameters loaded:");
        ROS_INFO("  Tag size: %f", opt_tagSize_);
        ROS_INFO("  Convergence threshold: %f", convergence_threshold_);
    }
    
    void initializeViSP() {
        // 相机内参
        /*
        double px, py, u0, v0;
        nh_.param("camera_px", px, 600.75);
        nh_.param("camera_py", py, 601.0);
        nh_.param("camera_u0", u0, 325.882);
        nh_.param("camera_v0", v0, 250.568);
        cam_.initPersProjWithoutDistortion(px, py, u0, v0);*/
        //设置相机内参
    	//vpCameraParameters cam_(600.750,601.000,325.882,250.568);
        cam_.initPersProjWithoutDistortion(600.750, 601.000, 325.882, 250.568);
        // 图像
        int image_width, image_height;
        nh_.param("image_width", image_width, 640);
        nh_.param("image_height", image_height, 480);
        I_.init(image_height, image_width);
        display_.init(I_, 0, 0, "MPC-IBVS Camera View");
        
        // AprilTag检测器
        detector_.setAprilTagFamily(vpDetectorAprilTag::TAG_36h11);
        detector_.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
        detector_.setDisplayTag(display_tag_);
        detector_.setAprilTagQuadDecimate(opt_quad_decimate_);
        
        // 相机到末端执行器的变换
        eMc_ = vpHomogeneousMatrix({-0.9988679632, -0.0266672272, -0.0393910032, 32.6903893873/1000,
                                    0.0271749617, -0.999553647,  -0.0124108139, 67.2805551257/1000,
                                    -0.0390424589, -0.0134672134,  0.9991467963, 27.4906771353/1000,
                                    0,0,0,1});
        
        // 期望位姿
        cdMo_ = vpHomogeneousMatrix(vpTranslationVector(0, 0, opt_tagSize_ * 6),
                                   vpRotationMatrix({1,0,0,0,-1,0,0,0,-1}));
        
        // 特征点
        points_.resize(4);
        points_[0].setWorldCoordinates(-opt_tagSize_/2, -opt_tagSize_/2, 0);
        points_[1].setWorldCoordinates(opt_tagSize_/2, -opt_tagSize_/2, 0);
        points_[2].setWorldCoordinates(opt_tagSize_/2, opt_tagSize_/2, 0);
        points_[3].setWorldCoordinates(-opt_tagSize_/2, opt_tagSize_/2, 0);
        
        
        p_.resize(4); 	//只分配空间,用于存储当前时刻图像特征
        pd_.resize(4);	//只分配空间,用于存储期望图像特征
        
        ROS_INFO("ViSP initialized with 4 features");
    }
    
    void initializeMPC(const vpHomogeneousMatrix& cMo) {
        MPCConfig config;
        // 从参数服务器加载MPC参数
        nh_.param("mpc_Np", config.Np, 6);
        nh_.param("mpc_Ts", config.Ts, 0.033);
        nh_.param("mpc_v_max", config.v_max, 2.5);
        nh_.param("mpc_w_max", config.w_max, 2.5);
        
        // 交互矩阵类型
        int interaction_type_int = 0;  // 默认MEAN
        nh_.param("mpc_interaction_type", interaction_type_int, 0);
        config.interaction_type = static_cast<vpServo::vpServoIteractionMatrixType>(interaction_type_int);
        
        // 图像尺寸
        nh_.param("image_width", config.image_width, 640.0);
        nh_.param("image_height", config.image_height, 480.0);
        
        // 相机内参
        nh_.param("camera_px", config.fx, 600.75);
        nh_.param("camera_py", config.fy, 601.0);
        nh_.param("camera_u0", config.cx, 325.882);
        nh_.param("camera_v0", config.cy, 250.568);
        
        // 权重矩阵
        double Q_scale, R_scale;
        nh_.param("mpc_Q_scale", Q_scale, 2.0);//Q越大,误差越小
        nh_.param("mpc_R_scale", R_scale, 10.0); //R越小,控制输入越大
        config.Q = Eigen::Matrix<double, 8, 8>::Identity() * Q_scale;
        config.R = Eigen::Matrix<double, 6, 6>::Identity() * R_scale;
        
        // 创建MPC控制器对象
        mpc_controller_ = std::make_unique<MPC_IBVS_Controller>(config);
        
        ROS_INFO("MPC Controller initialized:");
        ROS_INFO("  Prediction horizon: %d", config.Np);
        ROS_INFO("  Sampling time: %f", config.Ts);
        ROS_INFO("  Max linear velocity: %f", config.v_max);
        ROS_INFO("  Max angular velocity: %f", config.w_max);
        ROS_INFO("  Interaction matrix type: %d", interaction_type_int);
        ROS_INFO("  Camera fx: %f, fy: %f", config.fx, config.fy);
    }
    
    void initializePlotter() {
        plotter_ = new vpPlot(2, static_cast<int>(250 * 2), 500, 
                             static_cast<int>(I_.getWidth()) + 80, 10, 
                             "MPC-IBVS Real-time Monitoring");
        plotter_->setTitle(0, "Visual Features Error");
        plotter_->setTitle(1, "Camera Velocities");
        
        plotter_->initGraph(0, 8);
        plotter_->initGraph(1, 6);
        
        plotter_->setLegend(0, 0, "error_feat_p1_x");
        plotter_->setLegend(0, 1, "error_feat_p1_y");
        plotter_->setLegend(0, 2, "error_feat_p2_x");
        plotter_->setLegend(0, 3, "error_feat_p2_y");
        plotter_->setLegend(0, 4, "error_feat_p3_x");
        plotter_->setLegend(0, 5, "error_feat_p3_y");
        plotter_->setLegend(0, 6, "error_feat_p4_x");
        plotter_->setLegend(0, 7, "error_feat_p4_y");
        
        plotter_->setLegend(1, 0, "vc_x");
        plotter_->setLegend(1, 1, "vc_y");
        plotter_->setLegend(1, 2, "vc_z");
        plotter_->setLegend(1, 3, "wc_x");
        plotter_->setLegend(1, 4, "wc_y");
        plotter_->setLegend(1, 5, "wc_z");
    }
    
    void processImage(const sensor_msgs::ImageConstPtr& img_msg) {
        // 将ROS图像转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
        cv::Mat cv_img = cv_ptr->image;
        vpImageConvert::convert(cv_img, I_);
        
        // 显示图像
        display_.display(I_);
        display_.flush(I_);
        
        // 检测AprilTag
        std::vector<vpHomogeneousMatrix> cMo_vec;
        detector_.detect(I_, opt_tagSize_, cam_, cMo_vec);
        
        // 检查标签是否存在
        if (cMo_vec.size() == 1) {
            vpHomogeneousMatrix cMo = cMo_vec[0];
            
            // 处理首次检测
            if (first_time_) {
                handleFirstDetection(cMo);
                
                // 第一次检测到标签时初始化MPC控制器
                if (!mpc_controller_) {
                    initializeMPC(cMo);
                }
                
                first_time_ = false;
            }
            
            // 获取4个角点
            std::vector<vpImagePoint> corners = detector_.getPolygon(0);
            
            if (corners.size() == 4) {
                // 从角点更新当前特征
                updateCurrentFeatures(corners, cMo);
                
                // 如果未初始化，则初始化期望特征
                if (!pd_initialized_) {
                    initializeDesiredFeatures();
                    pd_initialized_ = true;
                }
                
                // 使用MPC计算控制输入
                Eigen::VectorXd v_c_mpc = mpc_controller_->solveMPC(p_, pd_, tau_prev_);
                
                // 更新控制历史
                tau_prev_ = v_c_mpc;
                
                // 转换为ViSP格式
                vpColVector v_c(6);
                for (int i = 0; i < 6; i++) {
                    v_c[i] = v_c_mpc(i);
                }
                
                // 计算误差
                double error_norm = computeErrorNorm();
                ROS_INFO("MPC Error norm: %.6f", error_norm);
                
                // 检查收敛性
                if (error_norm < convergence_threshold_) {
                    has_converged_ = true;
                    ROS_INFO("Convergence achieved! Final error: %.6f", error_norm);
                }
                
                // 发布控制命令
                publishControl(v_c);
                
                // 发布期望特征
                publishDesiredFeatures();
                
                // 可视化
                if (opt_plot_) {
                    visualize(v_c);
                }
            } else {
                ROS_WARN("Expected 4 corners, got %lu", corners.size());
            }
        } else {
            ROS_WARN_THROTTLE(1.0, "No AprilTag detected");
        }
        
        // 显示信息
        std::stringstream ss;
        ss << "MPC-IBVS Control | Error: " << std::fixed << std::setprecision(4) 
           << computeErrorNorm();
        vpDisplay::displayText(I_, 20, 20, ss.str(), vpColor::red);
        vpDisplay::displayText(I_, 40, 20, "Press 'q' to quit", vpColor::red);
        
        if (vpDisplay::getClick(I_, false)) {
            has_converged_ = true;
        }
    }
    
    void handleFirstDetection(const vpHomogeneousMatrix& cMo) {
        std::vector<vpHomogeneousMatrix> v_oMo(2), v_cdMc(2);
        v_oMo[1].buildFrom(0, 0, 0, 0, 0, M_PI);
        
        for (size_t i = 0; i < 2; i++) {
            v_cdMc[i] = cdMo_ * v_oMo[i] * cMo.inverse();
        }
        
        if (std::fabs(v_cdMc[0].getThetaUVector().getTheta()) < 
            std::fabs(v_cdMc[1].getThetaUVector().getTheta())) {
            oMo_ = v_oMo[0];
        } else {
            oMo_ = v_oMo[1];
        }
        
        ROS_INFO_STREAM("Selected oMo:\n" << oMo_);
    }
    
    void updateCurrentFeatures(const std::vector<vpImagePoint>& corners, 
                               const vpHomogeneousMatrix& cMo) {
        for (size_t i = 0; i < corners.size(); i++) {
            //构造p_当前时刻期望特征
            vpFeatureBuilder::create(p_[i], cam_, corners[i]);
            
            // 计算当前深度（使用当前位姿）
            vpColVector cp;
            points_[i].changeFrame(cMo, cp);
            p_[i].set_Z(cp[2]);
            
            ROS_DEBUG("Current feature %zu: (%.3f, %.3f, depth=%.3f)", 
                     i, p_[i].get_x(), p_[i].get_y(), p_[i].get_Z());
        }
    }
    
    void initializeDesiredFeatures() {
        for (size_t i = 0; i < points_.size(); i++) {
            vpColVector cP, p_temp;
            
            //把points投影到期望相机下的三维坐标
            points_[i].changeFrame(cdMo_ * oMo_, cP);
            
            // 投影到归一化平面
            points_[i].projection(cP, p_temp);
            
            // 设置期望特征
            pd_[i].set_x(p_temp[0]);    // 归一化x坐标
            pd_[i].set_y(p_temp[1]);    // 归一化y坐标
            pd_[i].set_Z(cP[2]);        // 期望深度（使用期望位姿计算）
            
            ROS_DEBUG("Desired feature %zu: (%.3f, %.3f, depth=%.3f)", 
                     i, pd_[i].get_x(), pd_[i].get_y(), pd_[i].get_Z());
        }
        
        ROS_INFO("Desired features initialized with depth from cdMo * oMo");
    }
    
    double computeErrorNorm() const {
        double error = 0.0;
        for (size_t i = 0; i < p_.size(); i++) {
            double dx = p_[i].get_x() - pd_[i].get_x();
            double dy = p_[i].get_y() - pd_[i].get_y();
            error += dx * dx + dy * dy;
        }
        return sqrt(error);
    }
    
    void publishDesiredFeatures() {
        if (!pd_initialized_) return;
        
        // 发布每个期望位置
        for (size_t i = 0; i < pd_.size(); i++) {
            geometry_msgs::Point point_msg;
            point_msg.x = pd_[i].get_x();  // 归一化x坐标
            point_msg.y = pd_[i].get_y();  // 归一化y坐标
            point_msg.z = pd_[i].get_Z();  // 深度
            
            // 发布期望位置
            pd_pub_.publish(point_msg);
            
            ROS_DEBUG_STREAM("Published target position " << i << ": (" 
                           << point_msg.x << ", " << point_msg.y << ", " << point_msg.z << ")");
        }
        
        // 发布期望像素坐标
        lys_visp_demo::PixelCoordinates pixel_msg;
        pixel_msg.pixel.clear();
        for (size_t i = 0; i < points_.size(); i++) {
            vpColVector cP, p_temp;
            points_[i].changeFrame(cdMo_ * oMo_, cP);
            points_[i].projection(cP, p_temp);
            // 转换为像素坐标
            vpImagePoint ip;
            vpMeterPixelConversion::convertPoint(cam_, p_temp[0], p_temp[1], ip);
            
            // 将像素坐标添加到消息
            pixel_msg.pixel.push_back(static_cast<float>(ip.get_u()));
            pixel_msg.pixel.push_back(static_cast<float>(ip.get_v()));
        }
        
        // 发布期望像素坐标
        pixel_pub_.publish(pixel_msg);
        ROS_DEBUG("Published target pixel coordinates");
    }
    
    void publishControl(const vpColVector& v_c) {
        if (!send_velocities_) return;
        
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = v_c[0];
        twist_msg.linear.y = v_c[1];
        twist_msg.linear.z = v_c[2];
        twist_msg.angular.x = v_c[3];
        twist_msg.angular.y = v_c[4];
        twist_msg.angular.z = v_c[5];
        
        // 发布相机速度
        vel_pub_.publish(twist_msg);
        
        ROS_DEBUG_STREAM("Published MPC velocity: ["
                       << std::fixed << std::setprecision(6)
                       << twist_msg.linear.x << ", " << twist_msg.linear.y << ", "
                       << twist_msg.linear.z << ", " << twist_msg.angular.x << ", "
                       << twist_msg.angular.y << ", " << twist_msg.angular.z << "]");
    }
    
	void visualize(const vpColVector& v_c) {
		if (!opt_plot_ || !plotter_) return;
		
		// 方法1：创建vpColVector并逐个赋值
		vpColVector errors(8);
		for (size_t i = 0; i < 4; i++) {
		    errors[2*i] = p_[i].get_x() - pd_[i].get_x();
		    errors[2*i+1] = p_[i].get_y() - pd_[i].get_y();
		}
		
		plotter_->plot(0, iter_plot_, errors);
		plotter_->plot(1, iter_plot_, v_c);
		iter_plot_++;
	}
    
    void run() {
        ROS_INFO("Starting MPC-IBVS control loop...");
        ROS_INFO("Waiting for AprilTag detection to initialize MPC...");
        
        while (ros::ok() && !has_converged_) {
            // 阻塞并等待图像消息
            sensor_msgs::ImageConstPtr img_msg = ros::topic::waitForMessage<sensor_msgs::Image>(
                "camera/color/image_raw", nh_);
            
            if(!img_msg) {
                ROS_WARN("No image message received");
                continue;
            }
            
            if (!first_time_) {
                ROS_DEBUG("Received image...");
            }
            
            processImage(img_msg);
            
            ros::spinOnce();
        }
        
        ROS_INFO("MPC-IBVS Controller shutdown");
    }
    
    bool hasConverged() const { return has_converged_; }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "jr603_mpc_ibvs");
    ros::NodeHandle nh;
    
    try {
        // 创建MPC控制器对象
        JR603_MPC_IBVS controller(nh);
        // 运行视觉伺服控制循环
        controller.run();
    } catch (const std::exception& e) {
        ROS_ERROR("MPC-IBVS Controller error: %s", e.what());
        return 1;
    }
    
    return 0;
}
