#include "mpc_ibvs_controllerV3.hpp"
#include <ros/ros.h>
#include <iostream>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
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
#include <cstring>

class JR603_MPC_IBVS {
private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Publisher pd_pub_;
    ros::Publisher pixel_pub_;
    
    std::unique_ptr<MPC_IBVS_Controller> mpc_controller_;
    
    vpCameraParameters cam_;
    vpImage<unsigned char> I_;
    vpDisplayX display_;
    vpDetectorAprilTag detector_;
    
    std::vector<vpFeaturePoint> p_, pd_;
    std::vector<vpPoint> points_;
    
    vpHomogeneousMatrix eMc_;
    vpHomogeneousMatrix cdMo_;
    vpHomogeneousMatrix oMo_;
    
    double opt_tagSize_;
    bool display_tag_;
    int opt_quad_decimate_;
    bool opt_plot_;
    double convergence_threshold_;
    
    bool has_converged_;
    bool send_velocities_;
    bool first_time_;
    
    vpPlot* plotter_;
    int iter_plot_;
    
    Eigen::VectorXd tau_prev_;
    bool pd_initialized_;
    
    // 视野约束参数
    double image_width_, image_height_;
    double margin_;
    
public:
    JR603_MPC_IBVS(ros::NodeHandle& nh) 
        : nh_(nh), 
          has_converged_(false),
          first_time_(true),
          plotter_(nullptr),
          iter_plot_(0),
          pd_initialized_(false) {
          
        // 普通变量赋值
        opt_tagSize_ = 0.0815;
        display_tag_ = true;
        opt_quad_decimate_ = 2;
        opt_plot_ = true;
        convergence_threshold_ = 0.001;
        send_velocities_ = true;
        
        // 视野约束参数
        image_width_ = 640.0;
        image_height_ = 480.0;
        margin_ = 20.0;  // 像素边界
        
        // ROS Publisher初始化
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/camera_velocity", 10);
        pd_pub_ = nh_.advertise<geometry_msgs::Point>("/pd_publish", 10);
        pixel_pub_ = nh_.advertise<lys_visp_demo::PixelCoordinates>("/image_pixel", 10);
        
        // 读取参数
        loadParameters();
        // 初始化VISP
        initializeViSP();
        // 初始化绘图
        if (opt_plot_) {
            initializePlotter();
        }
        // 读取上一时刻控制量
        tau_prev_ = Eigen::VectorXd::Zero(6);
        
        ROS_INFO("JR603 MPC-IBVS Controller initialized");
    }
    
    ~JR603_MPC_IBVS() {
        if (plotter_) {
            delete plotter_;
            plotter_ = nullptr;
        }
    }
    
    void loadParameters() {
        nh_.param("tag_size", opt_tagSize_, opt_tagSize_);
        nh_.param("display_tag", display_tag_, display_tag_);
        nh_.param("quad_decimate", opt_quad_decimate_, opt_quad_decimate_);
        nh_.param("plot", opt_plot_, opt_plot_);
        nh_.param("convergence_threshold", convergence_threshold_, convergence_threshold_);
        nh_.param("send_velocities", send_velocities_, send_velocities_);
        
        // 视野约束参数
        nh_.param("image_width", image_width_, image_width_);
        nh_.param("image_height", image_height_, image_height_);
        nh_.param("fov_margin", margin_, margin_);
        
        ROS_INFO("Parameters loaded:");
        ROS_INFO("  Tag size: %f", opt_tagSize_);
        ROS_INFO("  Convergence threshold: %f", convergence_threshold_);
        ROS_INFO("  Image size: %.0fx%.0f, Margin: %.1f", image_width_, image_height_, margin_);
    }
    
    void initializeViSP() {
        try {
            cam_.initPersProjWithoutDistortion(600.750, 601.000, 325.882, 250.568);
            
            int image_width = 640, image_height = 480;
            nh_.param("image_width", image_width, image_width);
            nh_.param("image_height", image_height, image_height);
            
            I_.init(image_height, image_width);
            display_.init(I_, 0, 0, "MPC-IBVS Camera View");
            
            detector_.setAprilTagFamily(vpDetectorAprilTag::TAG_36h11);
            detector_.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
            detector_.setDisplayTag(display_tag_);
            detector_.setAprilTagQuadDecimate(opt_quad_decimate_);
            
            // 修正矩阵初始化方式
            std::vector<double> data = {
                -0.9988679632, -0.0266672272, -0.0393910032, 32.6903893873/1000,
                0.0271749617, -0.999553647,  -0.0124108139, 67.2805551257/1000,
                -0.0390424589, -0.0134672134,  0.9991467963, 27.4906771353/1000,
                0,0,0,1
            };
            eMc_ = vpHomogeneousMatrix(data);
            
            vpTranslationVector translation(0, 0, opt_tagSize_ * 6);
            vpRotationMatrix rotation({1,0,0,0,-1,0,0,0,-1});
            cdMo_ = vpHomogeneousMatrix(translation, rotation);
            
            points_.resize(4);
            points_[0].setWorldCoordinates(-opt_tagSize_/2, -opt_tagSize_/2, 0);
            points_[1].setWorldCoordinates(opt_tagSize_/2, -opt_tagSize_/2, 0);
            points_[2].setWorldCoordinates(opt_tagSize_/2, opt_tagSize_/2, 0);
            points_[3].setWorldCoordinates(-opt_tagSize_/2, opt_tagSize_/2, 0);
            
            p_.resize(4);
            pd_.resize(4);
            
            ROS_INFO("ViSP initialized with 4 features");
        } catch (const std::exception& e) {
            ROS_ERROR("ViSP initialization failed: %s", e.what());
            throw;
        }
    }
    
    void initializePlotter() {
        try {
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
            
            ROS_INFO("Plotter initialized");
        } catch (const std::exception& e) {
            ROS_ERROR("Plotter initialization failed: %s", e.what());
            plotter_ = nullptr;
        }
    }
    
    void initializeMPC(const vpHomogeneousMatrix& cMo) {
        try {
            ROS_INFO("here is initializeMPC");
            MPCConfig config;
            
            nh_.param("mpc_Np", config.Np, 6);
            nh_.param("mpc_Ts", config.Ts, 0.1);
            nh_.param("mpc_v_max", config.v_max, 0.8);
            nh_.param("mpc_w_max", config.w_max, 0.8);
            
            int interaction_type_int = 0;
            nh_.param("mpc_interaction_type", interaction_type_int, 0);
            config.interaction_type = static_cast<vpServo::vpServoIteractionMatrixType>(interaction_type_int);
            
            nh_.param("image_width", config.image_width, 640.0);
            nh_.param("image_height", config.image_height, 480.0);
            
            nh_.param("camera_px", config.fx, 600.75);
            nh_.param("camera_py", config.fy, 601.0);
            nh_.param("camera_u0", config.cx, 325.882);
            nh_.param("camera_v0", config.cy, 250.568);
            
            double Q_scale = 1.0, R_scale = 1.0;
            nh_.param("mpc_Q_scale", Q_scale, 1.0);
            nh_.param("mpc_R_scale", R_scale, 1.0);
            
            config.Q = Eigen::Matrix<double, 8, 8>::Identity() * Q_scale;
            config.R = Eigen::Matrix<double, 6, 6>::Identity() * R_scale;
            
            // 设置视野约束参数
            config.image_width = image_width_;
            config.image_height = image_height_;
            config.fov_margin = margin_;
            
            ROS_INFO("Final config: Np=%d, Ts=%.3f, v_max=%.2f, w_max=%.2f",
                 config.Np, config.Ts, config.v_max, config.w_max);
            ROS_INFO("Q_scale=%.1f, R_scale=%.1f", Q_scale, R_scale);
            ROS_INFO("FOV constraints: %.0fx%.0f, margin=%.1f", 
                     config.image_width, config.image_height, config.fov_margin);
        
            ROS_INFO("next into create mpc_controller_");    
            mpc_controller_ = std::make_unique<MPC_IBVS_Controller>(config);
            ROS_INFO("MPC Controller initialized successfully");
            
        } catch (const std::exception& e) {
            ROS_ERROR("MPC initialization failed: %s", e.what());
            throw;
        }
    }
    
    void processImage(const sensor_msgs::ImageConstPtr& img_msg) {
        try {
            // ROS图像转换为VISP图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
            cv::Mat cv_img = cv_ptr->image;
            vpImageConvert::convert(cv_img, I_);
            // 显示刷新图像
            display_.display(I_);
            display_.flush(I_);
            // Apriltag检测
            std::vector<vpHomogeneousMatrix> cMo_vec;
            detector_.detect(I_, opt_tagSize_, cam_, cMo_vec);
            
            if (cMo_vec.size() == 1) {
                // 提取第一个Tag的位姿
                vpHomogeneousMatrix cMo = cMo_vec[0];
                
                if (first_time_) {
                    ROS_INFO_STREAM("next into handleFirstDetection");
                    handleFirstDetection(cMo);
                    if (!mpc_controller_) {
                        ROS_INFO_STREAM("next into initializeMPC");
                        initializeMPC(cMo);
                    }
                    first_time_ = false;
                }
                
                ROS_INFO("the next is detector_.getPolygon");
                std::vector<vpImagePoint> corners = detector_.getPolygon(0);
                ROS_INFO("the next is updateCurrentFeatures");
                if (corners.size() == 4) {
                    updateCurrentFeatures(corners, cMo);
                    
                    if (!pd_initialized_) {
                        initializeDesiredFeatures();
                        pd_initialized_ = true;
                    }
                    
                    // 检查视野约束
                    if (checkFOVConstraints()) {
                        ROS_WARN("Features approaching FOV boundary, applying constraints");
                    }
                    
                    ROS_INFO("the next is solveMPC");
                    Eigen::VectorXd v_c_mpc = mpc_controller_->solveMPC(p_, pd_, tau_prev_);
                    tau_prev_ = v_c_mpc;
                    
                    vpColVector v_c(6);
                    for (int i = 0; i < 6; i++) {
                        v_c[i] = v_c_mpc(i);
                    }
                    
                    double error_norm = computeErrorNorm();
                    ROS_INFO("MPC Error norm: %.6f", error_norm);
                    
                    if (error_norm < convergence_threshold_) {
                        has_converged_ = true;
                        ROS_INFO("Convergence achieved! Final error: %.6f", error_norm);
                    }
                    
                    publishControl(v_c);
                    publishDesiredFeatures();
                    
                    if (opt_plot_ && plotter_) {
                        visualize(v_c);
                    }
                } else {
                    ROS_WARN("Expected 4 corners, got %lu", corners.size());
                }
            } else {
                ROS_WARN_THROTTLE(1.0, "No AprilTag detected");
            }
            
            std::stringstream ss;
            ss << "MPC-IBVS Control | Error: " << std::fixed << std::setprecision(4) 
               << computeErrorNorm();
            vpDisplay::displayText(I_, 20, 20, ss.str(), vpColor::red);
            vpDisplay::displayText(I_, 40, 20, "Press 'q' to quit", vpColor::red);
            
            if (vpDisplay::getClick(I_, false)) {
                has_converged_ = true;
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Image processing failed: %s", e.what());
        }
    }
    
    bool checkFOVConstraints() {
        bool near_boundary = false;
        
        for (size_t i = 0; i < p_.size(); i++) {
            vpImagePoint ip;
            // 将特征点从米转换为像素坐标
            vpMeterPixelConversion::convertPoint(cam_, p_[i].get_x(), p_[i].get_y(), ip);
            
            double u = ip.get_u();
            double v = ip.get_v();
            
            // 检查是否接近图像边界
            if (u < margin_ || u > image_width_ - margin_ || 
                v < margin_ || v > image_height_ - margin_) {
                near_boundary = true;
                ROS_WARN_THROTTLE(1.0, "Feature %zu near boundary: (%.1f, %.1f)", i, u, v);
            }
        }
        
        return near_boundary;
    }
    
    void handleFirstDetection(const vpHomogeneousMatrix& cMo) {
        try {
            std::vector<vpHomogeneousMatrix> v_oMo(2), v_cdMc(2);
            
            v_oMo[0] = vpHomogeneousMatrix();  // 单位矩阵
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
            
        } catch (const std::exception& e) {
            ROS_ERROR("First detection handling failed: %s", e.what());
            throw;
        }
    }
    
    void updateCurrentFeatures(const std::vector<vpImagePoint>& corners, 
                               const vpHomogeneousMatrix& cMo) {
        for (size_t i = 0; i < corners.size(); i++) {
            vpFeatureBuilder::create(p_[i], cam_, corners[i]);
            
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
            
            points_[i].changeFrame(cdMo_ * oMo_, cP);
            points_[i].projection(cP, p_temp);
            
            pd_[i].set_x(p_temp[0]);
            pd_[i].set_y(p_temp[1]);
            pd_[i].set_Z(cP[2]);
            
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
        
        for (size_t i = 0; i < pd_.size(); i++) {
            geometry_msgs::Point point_msg;
            point_msg.x = pd_[i].get_x();
            point_msg.y = pd_[i].get_y();
            point_msg.z = pd_[i].get_Z();
            pd_pub_.publish(point_msg);
        }
        
        lys_visp_demo::PixelCoordinates pixel_msg;
        pixel_msg.pixel.clear();
        for (size_t i = 0; i < points_.size(); i++) {
            vpColVector cP, p_temp;
            points_[i].changeFrame(cdMo_ * oMo_, cP);
            points_[i].projection(cP, p_temp);
            vpImagePoint ip;
            vpMeterPixelConversion::convertPoint(cam_, p_temp[0], p_temp[1], ip);
            
            pixel_msg.pixel.push_back(static_cast<float>(ip.get_u()));
            pixel_msg.pixel.push_back(static_cast<float>(ip.get_v()));
        }
        
        pixel_pub_.publish(pixel_msg);
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
        
        vel_pub_.publish(twist_msg);
        
        ROS_DEBUG_STREAM("Published MPC velocity: ["
                       << std::fixed << std::setprecision(6)
                       << twist_msg.linear.x << ", " << twist_msg.linear.y << ", "
                       << twist_msg.linear.z << ", " << twist_msg.angular.x << ", "
                       << twist_msg.angular.y << ", " << twist_msg.angular.z << "]");
    }
    
    void visualize(const vpColVector& v_c) {
        if (!opt_plot_ || !plotter_) return;
        
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
        ROS_INFO("Into run()");
        ROS_INFO("Starting MPC-IBVS control loop...");
        
        try {
            while (ros::ok() && !has_converged_) {
                sensor_msgs::ImageConstPtr img_msg = 
                    ros::topic::waitForMessage<sensor_msgs::Image>("camera/color/image_raw", nh_);
                
                if (!img_msg) {
                    ROS_WARN_THROTTLE(1.0, "No image message received");
                    continue;
                }
                ROS_INFO("next into processImage()");
                processImage(img_msg);
                ros::spinOnce();
                
                ros::Duration(0.001).sleep();
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Control loop error: %s", e.what());
        }
        
        ROS_INFO("MPC-IBVS Controller shutdown");
    }
    
    bool hasConverged() const { return has_converged_; }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "jr603_mpc_ibvs");
    
    try {
        ros::NodeHandle nh;
        JR603_MPC_IBVS controller(nh);
        ROS_INFO("next into controller.run()");
        controller.run();
    } catch (const std::exception& e) {
        ROS_ERROR("MPC-IBVS Controller fatal error: %s", e.what());
        return 1;
    }
    
    return 0;
}
