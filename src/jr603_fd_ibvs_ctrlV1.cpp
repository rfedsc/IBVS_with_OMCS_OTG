#include <ros/ros.h>
#include <iostream>
#include <iomanip>  
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
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// 添加模糊控制头文件
#include <fl/Headers.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <std_msgs/Float32MultiArray.h>
// 新增：包含Float32头文件
#include <std_msgs/Float32.h>
#include <lys_visp_demo/PixelCoordinates.h>
#include <deque>
#include <numeric>
#include <cmath>
#include <vector>
#include <algorithm>

// 衰减策略枚举
enum DecayStrategy {
    NO_DECAY = 0,        // 无衰减
    POLYNOMIAL_DECAY = 1,// 多项式衰减
    EXPONENTIAL_DECAY = 2// 指数衰减
};

// 模糊控制器类
class FuzzyGainController {
private:
    fl::Engine* engine;
    fl::InputVariable* alpha;
    fl::InputVariable* distance;
    fl::OutputVariable* gain_k;
    fl::OutputVariable* gain_n;
    
public:
    FuzzyGainController() {
        engine = new fl::Engine("FuzzyGainController");
        
        // 输入变量：方向角（归一化到[-1,1]）
        alpha = new fl::InputVariable;
        alpha->setName("alpha");
        alpha->setRange(-1.0, 1.0);
        
        // 方向角的隶属函数（8个方向区间）
        alpha->addTerm(new fl::Trapezoid("D1", -1.0, -0.875, -0.75));
        alpha->addTerm(new fl::Trapezoid("D2", -0.75, -0.625, -0.5));
        alpha->addTerm(new fl::Trapezoid("D3", -0.5, -0.375, -0.25));
        alpha->addTerm(new fl::Trapezoid("D4", -0.25, -0.125, 0));
        alpha->addTerm(new fl::Trapezoid("D5", 0, 0.125, 0.25));
        alpha->addTerm(new fl::Trapezoid("D6", 0.25, 0.375, 0.5));
        alpha->addTerm(new fl::Trapezoid("D7", 0.5, 0.625, 0.75));
        alpha->addTerm(new fl::Trapezoid("D8", 0.75, 0.875, 1.0));
        
        // 输入变量：归一化距离
        distance = new fl::InputVariable;
        distance->setName("distance");
        distance->setRange(0, 1.0);
        distance->addTerm(new fl::Triangle("CLOSE", 0, 0.15, 0.3));
        distance->addTerm(new fl::Triangle("MEDIUM", 0.2, 0.5, 0.8));
        distance->addTerm(new fl::Triangle("FAR", 0.7, 0.85, 1.0));
        
        // 输出变量：K（x方向增益系数）
        gain_k = new fl::OutputVariable;
        gain_k->setName("K");
        gain_k->setRange(0.5, 2.0);
        gain_k->setDefaultValue(1.0);
        gain_k->addTerm(new fl::Gaussian("LOW", 0.5, 0.1));
        gain_k->addTerm(new fl::Gaussian("MEDIUM", 1.0, 0.1));
        gain_k->addTerm(new fl::Gaussian("HIGH", 1.5, 0.1));
        
        // 输出变量：N（y方向增益系数）
        gain_n = new fl::OutputVariable;
        gain_n->setName("N");
        gain_n->setRange(0.5, 2.0);
        gain_n->setDefaultValue(1.0);
        gain_n->addTerm(new fl::Gaussian("LOW", 0.5, 0.1));
        gain_n->addTerm(new fl::Gaussian("MEDIUM", 1.0, 0.1));
        gain_n->addTerm(new fl::Gaussian("HIGH", 1.5, 0.1));
        
        // 添加到引擎
        engine->addInputVariable(alpha);
        engine->addInputVariable(distance);
        engine->addOutputVariable(gain_k);
        engine->addOutputVariable(gain_n);
        
        // 模糊规则（与MATLAB代码对应）
        fl::RuleBlock* ruleBlock = new fl::RuleBlock;
        
        // D1方向规则
        ruleBlock->addRule(fl::Rule::parse("if alpha is D1 and distance is FAR then K is HIGH and N is LOW", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D1 and distance is MEDIUM then K is HIGH and N is MEDIUM", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D1 and distance is CLOSE then K is MEDIUM and N is LOW", engine));
        
        // D2方向规则
        ruleBlock->addRule(fl::Rule::parse("if alpha is D2 and distance is FAR then K is MEDIUM and N is HIGH", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D2 and distance is MEDIUM then K is MEDIUM and N is MEDIUM", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D2 and distance is CLOSE then K is LOW and N is HIGH", engine));
        
        // D3方向规则
        ruleBlock->addRule(fl::Rule::parse("if alpha is D3 and distance is FAR then K is LOW and N is HIGH", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D3 and distance is MEDIUM then K is LOW and N is HIGH", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D3 and distance is CLOSE then K is LOW and N is MEDIUM", engine));
        
        // D4方向规则
        ruleBlock->addRule(fl::Rule::parse("if alpha is D4 and distance is FAR then K is HIGH and N is LOW", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D4 and distance is MEDIUM then K is HIGH and N is LOW", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D4 and distance is CLOSE then K is MEDIUM and N is LOW", engine));
        
        // D5方向规则
        ruleBlock->addRule(fl::Rule::parse("if alpha is D5 and distance is FAR then K is HIGH and N is LOW", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D5 and distance is MEDIUM then K is HIGH and N is LOW", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D5 and distance is CLOSE then K is MEDIUM and N is LOW", engine));
        
        // D6方向规则
        ruleBlock->addRule(fl::Rule::parse("if alpha is D6 and distance is FAR then K is LOW and N is HIGH", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D6 and distance is MEDIUM then K is LOW and N is HIGH", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D6 and distance is CLOSE then K is LOW and N is MEDIUM", engine));
        
        // D7方向规则
        ruleBlock->addRule(fl::Rule::parse("if alpha is D7 and distance is FAR then K is MEDIUM and N is HIGH", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D7 and distance is MEDIUM then K is MEDIUM and N is MEDIUM", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D7 and distance is CLOSE then K is LOW and N is HIGH", engine));
        
        // D8方向规则
        ruleBlock->addRule(fl::Rule::parse("if alpha is D8 and distance is FAR then K is HIGH and N is LOW", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D8 and distance is MEDIUM then K is HIGH and N is MEDIUM", engine));
        ruleBlock->addRule(fl::Rule::parse("if alpha is D8 and distance is CLOSE then K is MEDIUM and N is LOW", engine));
        
        engine->addRuleBlock(ruleBlock);
        
        // 配置引擎
        fl::TNorm* conjunction = new fl::Minimum;
        fl::SNorm* disjunction = new fl::Maximum;
        fl::TNorm* activation = new fl::Minimum;
        fl::SNorm* accumulation = new fl::Maximum;
        fl::Defuzzifier* defuzzifier = new fl::Centroid(100);
        
        engine->configure(conjunction, disjunction, activation, accumulation, defuzzifier, 0);
    }
    
    ~FuzzyGainController() {
        delete engine;
    }
    
    // 计算增益系数
    std::pair<double, double> computeGains(double current_sigma, double norm_distance) {
        // 设置输入值
        alpha->setValue(current_sigma);
        distance->setValue(norm_distance);
        
        // 处理模糊推理
        engine->process();
        
        // 获取输出值
        double k = gain_k->getValue();
        double n = gain_n->getValue();
        
        // 限制输出范围
        k = std::max(0.5, std::min(2.0, k));
        n = std::max(0.5, std::min(2.0, n));
        
        return std::make_pair(k, n);
    }
};

// 自适应增益控制器
class AdaptiveGainController {
private:
    double lambda_0;
    double lambda_inf;
    double lambda_prime_0;
    
public:
    AdaptiveGainController(double lambda0 = 1.5, double lambdaInf = 1.0, double lambdaPrime0 = 0.05)
        : lambda_0(lambda0), lambda_inf(lambdaInf), lambda_prime_0(lambdaPrime0) {}
    
    double computeGain(int iteration) {
        double t = iteration;
        return (lambda_0 - lambda_inf) * exp(-lambda_prime_0 * t) + lambda_inf;
    }
};

// 衰减函数计算器
class DecayFunction {
public:
    // 多项式衰减：1/(1 + 3 * iteration^5)
    static double polynomialDecay(int iteration) {
        if (iteration <= 0) return 1.0;
        return 1.0 / (1.0 + 3.0 * pow(static_cast<double>(iteration), 5.0));
    }
    
    // 指数衰减：exp(-4 * iteration)
    static double exponentialDecay(int iteration) {
        if (iteration <= 0) return 1.0;
        return exp(-4.0 * static_cast<double>(iteration));
    }
    
    // 通用衰减计算接口
    static double computeDecay(int iteration, DecayStrategy strategy) {
        switch (strategy) {
            case POLYNOMIAL_DECAY:
                return polynomialDecay(iteration);
            case EXPONENTIAL_DECAY:
                return exponentialDecay(iteration);
            case NO_DECAY:
            default:
                return 1.0;
        }
    }
};

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "jr603_ibvs_ctrl_fuzzy");
    ros::NodeHandle nh;
    
    // 创建控制器实例
    FuzzyGainController fuzzy_controller;
    AdaptiveGainController adaptive_controller(2.0, 1.0, 0.1);
    
    // 创建发布器
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/camera_velocity", 10);
    ros::Publisher feature_pub = nh.advertise<geometry_msgs::Point>("/pd_publish", 10);
    ros::Publisher pixel_pub = nh.advertise<lys_visp_demo::PixelCoordinates>("/image_pixel", 10);
    ros::Publisher error_pub = nh.advertise<std_msgs::Float32MultiArray>("/error_vector", 10);
    ros::Publisher gain_pub = nh.advertise<std_msgs::Float32MultiArray>("/gain_values", 10);
    // 修复：正确声明Float32发布器
    ros::Publisher decay_pub = nh.advertise<std_msgs::Float32>("/decay_value", 10);
    
    geometry_msgs::Twist twist_msg;
    geometry_msgs::Point point_msg;
    lys_visp_demo::PixelCoordinates pixel4_msg;
    pixel4_msg.pixel.clear();
    
    // 设置参数
    double opt_tagSize = 0.0815;
    bool display_tag = true;
    int opt_quad_decimate = 2;
    bool opt_adaptive_gain = false;
    bool opt_task_sequencing = true;
    double convergence_threshold = 0.0001;
    bool opt_plot = true;
    
    // 模糊控制参数
    double lambda_base = 2.0;
    bool use_fuzzy_control = true;
    bool use_diagonal_gain = true;
    
    // 衰减策略参数
    DecayStrategy decay_strategy = POLYNOMIAL_DECAY;  // 默认使用多项式衰减
    bool use_continuous_velocity = true;              // 是否启用连续速度控制
    
    // 从参数服务器读取配置
    nh.param("use_fuzzy_control", use_fuzzy_control, true);
    nh.param("use_diagonal_gain", use_diagonal_gain, true);
    nh.param("lambda_base", lambda_base, 2.0);
    nh.param("use_continuous_velocity", use_continuous_velocity, true);
    
    // 读取衰减策略配置（0:无衰减, 1:多项式, 2:指数）
    int decay_type = 1;
    nh.param("decay_strategy", decay_type, 1);
    decay_strategy = static_cast<DecayStrategy>(decay_type);
    
    // 定义相机内参
    vpCameraParameters cam(600.750, 601.000, 325.882, 250.568);
    vpImage<unsigned char> I(480, 640);
    vpDisplayX display;
    display.init(I, 0, 0, "Image with Fuzzy IBVS Control");
    
    // AprilTag检测器
    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setDisplayTag(display_tag);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);
    
    // 期望位姿
    vpHomogeneousMatrix cdMo(vpTranslationVector(0, 0, opt_tagSize * 6),
                            vpRotationMatrix({1, 0, 0, 0, -1, 0, 0, 0, -1}));
    
    vpHomogeneousMatrix oMo;
    std::vector<vpFeaturePoint> p(4), pd(4);
    std::vector<vpPoint> point(4);
    point[0].setWorldCoordinates(-opt_tagSize/2, -opt_tagSize/2, 0);
    point[1].setWorldCoordinates(opt_tagSize/2, -opt_tagSize/2, 0);
    point[2].setWorldCoordinates(opt_tagSize/2, opt_tagSize/2, 0);
    point[3].setWorldCoordinates(-opt_tagSize/2, opt_tagSize/2, 0);
    
    vpServo task;
    for (size_t i = 0; i < p.size(); i++) {
        task.addFeature(p[i], pd[i]);
    }
    
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::MEAN);
    task.setLambda(lambda_base);
    
    bool has_converged = false;
    bool send_velocities = true;
    double t_init_servo = vpTime::measureTimeMs();
    
    // 绘图器 - 修改：增加第4个窗口用于显示k和n增益
    vpPlot *plotter = nullptr;
    int iter_plot = 0;
    if (opt_plot) {
        // 修改：将窗口数量从3改为4，高度调整为250*4
        plotter = new vpPlot(4, static_cast<int>(250 * 4), 500, 
                            static_cast<int>(I.getWidth()) + 80, 10, "Real time curves plotter");
        plotter->setTitle(0, "Visual features error");
        plotter->setTitle(1, "Camera velocities");
        plotter->setTitle(2, "Decay value");
        // 新增：第4个窗口标题
        plotter->setTitle(3, "Gain K and N");
        
        plotter->initGraph(0, 8);
        plotter->initGraph(1, 6);
        plotter->initGraph(2, 1);  // 衰减值只有1条曲线
        // 新增：初始化第4个窗口的2条曲线（K和N）
        plotter->initGraph(3, 2);  
        
        // 误差曲线图例
        plotter->setLegend(0, 0, "error_feat_p1_x");
        plotter->setLegend(0, 1, "error_feat_p1_y");
        plotter->setLegend(0, 2, "error_feat_p2_x");
        plotter->setLegend(0, 3, "error_feat_p2_y");
        plotter->setLegend(0, 4, "error_feat_p3_x");
        plotter->setLegend(0, 5, "error_feat_p3_y");
        plotter->setLegend(0, 6, "error_feat_p4_x");
        plotter->setLegend(0, 7, "error_feat_p4_y");
        
        // 速度曲线图例
        plotter->setLegend(1, 0, "vc_x");
        plotter->setLegend(1, 1, "vc_y");
        plotter->setLegend(1, 2, "vc_z");
        plotter->setLegend(1, 3, "wc_x");
        plotter->setLegend(1, 4, "wc_y");
        plotter->setLegend(1, 5, "wc_z");
        
        // 衰减值曲线图例
        plotter->setLegend(2, 0, "decay_value");
        
        // 新增：K和N增益曲线图例
        plotter->setLegend(3, 0, "Gain K (x-direction)");
        plotter->setLegend(3, 1, "Gain N (y-direction)");
    }
    
    // 控制变量
    int iteration = 0;
    double last_error = 0;
    double current_center_x = 0, current_center_y = 0;
    double target_center_x = 0, target_center_y = 0;
    double k_gain = 1.0, n_gain = 1.0;
    
    // 连续速度控制所需变量
    vpColVector e_initial;          // 初始误差
    vpMatrix L_pinv_initial;        // 初始交互矩阵伪逆
    bool is_initialized = false;    // 初始值是否已初始化
    
    // 图像尺寸
    int image_width = 640;
    int image_height = 480;
    
    while (ros::ok() && !has_converged) {
        // 获取图像
        sensor_msgs::ImageConstPtr img_msg = 
            ros::topic::waitForMessage<sensor_msgs::Image>("camera/color/image_raw", nh);
        if (!img_msg) continue;
        
        // 转换为ViSP图像格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
        cv::Mat cv_img = cv_ptr->image;
        vpImageConvert::convert(cv_img, I);
        display.display(I);
        display.flush(I);
        
        // 检测AprilTag
        std::vector<vpHomogeneousMatrix> cMo_vec;
        detector.detect(I, opt_tagSize, cam, cMo_vec);
        std::vector<vpImagePoint> corners = detector.getPolygon(0);
        
        if (cMo_vec.size() == 1) {
            vpHomogeneousMatrix cMo = cMo_vec[0];
            static bool first_time = true;
            
            if (first_time) {
                // 初始化期望位置
                vpImagePoint ip;
                for (size_t i = 0; i < point.size(); i++) {
                    vpColVector cP, p_;
                    point[i].changeFrame(cdMo, cP);
                    point[i].projection(cP, p_);
                    pd[i].set_x(p_[0]);
                    pd[i].set_y(p_[1]);
                    pd[i].set_Z(cP[2]);
                    
                    // 计算期望特征点中心
                    vpMeterPixelConversion::convertPoint(cam, pd[i].get_x(), pd[i].get_y(), ip);
                    target_center_x += ip.get_u();
                    target_center_y += ip.get_v();
                    
                    // 发布期望特征点
                    point_msg.x = pd[i].get_x();
                    point_msg.y = pd[i].get_y();
                    point_msg.z = pd[i].get_Z();
                    feature_pub.publish(point_msg);
                    
                    // 收集像素坐标
                    float u = static_cast<float>(ip.get_u());
                    float v = static_cast<float>(ip.get_v());
                    pixel4_msg.pixel.push_back(u);
                    pixel4_msg.pixel.push_back(v);
                    
                    ROS_INFO_STREAM("Projected point " << i << ": x=" << p_[0] << ", y=" << p_[1]);
                    ROS_INFO_STREAM("pd["<<i<<"]:x="<<pd[i].get_x()<<",y="<<pd[i].get_y()<<",Z="<<pd[i].get_Z());
                    ROS_INFO_STREAM("ip is: (" << ip.get_u() << ", " << ip.get_v() << ")");
                }
                
                // 发布像素坐标消息
                if(pixel4_msg.pixel.size() == 8) {
                    pixel_pub.publish(pixel4_msg);
                    ROS_INFO_STREAM("Published pixel4_msg: " << pixel4_msg);
                    pixel4_msg.pixel.clear();
                }
                
                target_center_x /= 4.0;
                target_center_y /= 4.0;
                ROS_INFO("Target center: (%f, %f)", target_center_x, target_center_y);
                
                first_time = false;
            }
            
            // 计算当前特征点中心
            current_center_x = 0;
            current_center_y = 0;
            for (size_t i = 0; i < corners.size(); i++) {
                current_center_x += corners[i].get_u();
                current_center_y += corners[i].get_v();
            }
            current_center_x /= corners.size();
            current_center_y /= corners.size();
            
            // 计算方向角和距离
            double dx = target_center_x - current_center_x;
            double dy = target_center_y - current_center_y;
            double distance = sqrt(dx*dx + dy*dy) / sqrt(image_width*image_width + image_height*image_height);
            double alpha = atan2(dy, dx) / M_PI;  // 归一化到[-1,1]
            
            ROS_INFO("Alpha: %f, Distance: %f", alpha, distance);
            
            // 计算模糊增益
            std::pair<double, double> gains = fuzzy_controller.computeGains(alpha, distance);
            k_gain = gains.first;
            n_gain = gains.second;
            
            ROS_INFO("Fuzzy gains - K: %f, N: %f", k_gain, n_gain);
            
            // 发布增益值
            std_msgs::Float32MultiArray gain_msg;
            gain_msg.data = {static_cast<float>(k_gain), static_cast<float>(n_gain)};
            gain_pub.publish(gain_msg);
            
            // 更新视觉特征
            for (size_t i = 0; i < corners.size(); i++) {
                vpFeatureBuilder::create(p[i], cam, corners[i]);
                vpColVector cp;
                point[i].changeFrame(cMo, cp);
                p[i].set_Z(cp[2]);
                
                ROS_INFO_STREAM("p["<<i<<"]:x="<<p[i].get_x()<<",y="<<p[i].get_y()<<",Z="<<p[i].get_Z());
            }
            
            // 计算自适应增益
            double adaptive_lambda = 1.0;
            if (opt_adaptive_gain) {
                adaptive_lambda = adaptive_controller.computeGain(iteration);
            }
            
            // 计算控制律
            task.computeError();
            vpColVector e = task.getError();
            vpMatrix L = task.computeInteractionMatrix();
            vpMatrix L_pinv = L.pseudoInverse();
            
            // 初始化连续速度控制的初始值
            if (!is_initialized) {
                e_initial = e;
                L_pinv_initial = L_pinv;
                is_initialized = true;
                ROS_INFO("Initialized continuous velocity control variables");
            }
            
            // 计算衰减值
            double decay_value = DecayFunction::computeDecay(iteration, decay_strategy);
            
            // 修复：正确声明和使用Float32消息
            std_msgs::Float32 decay_msg;
            decay_msg.data = static_cast<float>(decay_value);
            decay_pub.publish(decay_msg);
            
            // 使用模糊增益矩阵 + 连续速度控制
            vpColVector v_c(6, 0.0);
            if (use_fuzzy_control && use_diagonal_gain) {
                // 创建对角增益矩阵
                vpMatrix Lambda_diag(6, 6, 0.0);
                Lambda_diag[0][0] = k_gain * lambda_base;
                Lambda_diag[1][1] = n_gain * lambda_base;
                //Lambda_diag[0][0] = 1.0 * lambda_base;
                //Lambda_diag[1][1] = 2.0 * lambda_base;
                //Lambda_diag[2][2] = 1.0 * lambda_base;
                Lambda_diag[3][3] = 1.0 * lambda_base;
                Lambda_diag[4][4] = 1.0 * lambda_base;
                Lambda_diag[5][5] = 1.0 * lambda_base;
                
                // 基础控制律
                vpColVector v_c_base = -Lambda_diag * L_pinv * e;
                
                // 连续速度控制项（衰减项）
                vpColVector v_c_continuous(6, 0.0);
                if (use_continuous_velocity && is_initialized) {
                    v_c_continuous = Lambda_diag * L_pinv_initial * e_initial * decay_value;
                }
                
                // 总控制律 = 基础控制律 + 连续速度控制项
                v_c = v_c_base + v_c_continuous;
            } else if (use_fuzzy_control) {
                // 使用标量模糊增益
                double fuzzy_lambda = (k_gain + n_gain) / 2.0 * lambda_base;
                
                // 基础控制律
                vpColVector v_c_base = -fuzzy_lambda * L_pinv * e;
                
                // 连续速度控制项
                vpColVector v_c_continuous(6, 0.0);
                if (use_continuous_velocity && is_initialized) {
                    v_c_continuous = fuzzy_lambda * L_pinv_initial * e_initial * decay_value;
                }
                
                v_c = v_c_base + v_c_continuous;
            } else {
                // 使用原始方法
                v_c = opt_adaptive_gain ? 
                      task.computeControlLaw((vpTime::measureTimeMs() - t_init_servo) / 1000.0) :
                      task.computeControlLaw();
            }
            
            // 计算误差
            double error = task.getError().sumSquare();
            
            // 发布误差向量
            std_msgs::Float32MultiArray error_msg;
            error_msg.data.clear();
            for (int i = 0; i < e.size(); i++) {
                error_msg.data.push_back(static_cast<float>(e[i]));
            }
            error_pub.publish(error_msg);
            
            ROS_INFO("Current error: %f, Decay value: %f", error, decay_value);
            ROS_INFO("Continuous velocity control: %s", use_continuous_velocity ? "ENABLED" : "DISABLED");
            
            // 绘制图表
            if (opt_plot) {
                plotter->plot(0, iter_plot, task.getError());
                plotter->plot(1, iter_plot, v_c);
                // 修复：vpPlot::plot需要4个参数（graphNum, curveNum, x, y）
                plotter->plot(2, 0, static_cast<double>(iter_plot), decay_value);
                
                // 新增：绘制K和N增益曲线
                plotter->plot(3, 0, static_cast<double>(iter_plot), k_gain);  // K增益曲线
                plotter->plot(3, 1, static_cast<double>(iter_plot), n_gain);  // N增益曲线
                
                iter_plot++;
            }
            
            // 检查收敛条件
            if (error < convergence_threshold) {
                has_converged = true;
                ROS_INFO("Control converged!");
            }
            
            if (!send_velocities) {
                v_c = 0;
            }
            
            // 设置并发布速度消息
            twist_msg.linear.x = v_c[0];
            twist_msg.linear.y = v_c[1];
            twist_msg.linear.z = v_c[2];
            twist_msg.angular.x = v_c[3];
            twist_msg.angular.y = v_c[4];
            twist_msg.angular.z = v_c[5];
            
            if (send_velocities) {
                vel_pub.publish(twist_msg);
                ROS_INFO_STREAM("Published velocity:["
                                <<std::fixed<<std::setprecision(6)
                                <<twist_msg.linear.x<<","<<twist_msg.linear.y<<","
                                <<twist_msg.linear.z<<","<<twist_msg.angular.x<<","
                                <<twist_msg.angular.y<<","<<twist_msg.angular.z<<"]");
            }
            
            iteration++;
            
            // 显示信息
            std::string decay_type_str = (decay_strategy == POLYNOMIAL_DECAY) ? "Polynomial" : 
                                        (decay_strategy == EXPONENTIAL_DECAY) ? "Exponential" : "None";
            
            vpDisplay::displayText(I, 20, 20, "Fuzzy IBVS with Continuous Velocity Control", vpColor::red);
            vpDisplay::displayText(I, 40, 20, 
                ("Alpha: " + std::to_string(alpha) + 
                 ", Distance: " + std::to_string(distance)).c_str(), 
                vpColor::red);
            vpDisplay::displayText(I, 60, 20, 
                ("Gain K: " + std::to_string(k_gain) + 
                 ", Gain N: " + std::to_string(n_gain)).c_str(), 
                vpColor::red);
            vpDisplay::displayText(I, 80, 20, 
                ("Error: " + std::to_string(error) + 
                 ", Decay: " + std::to_string(decay_value)).c_str(), 
                vpColor::red);
            vpDisplay::displayText(I, 100, 20, 
                ("Decay strategy: " + decay_type_str).c_str(), 
                vpColor::red);
            vpDisplay::displayText(I, 120, 20, 
                "Press 'q' to quit", vpColor::red);
            
            if (vpDisplay::getClick(I, false)) {
                break;
            }
        }
        
        ros::spinOnce();
    }
    
    // 发布零速度
    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = 0;
    vel_pub.publish(twist_msg);
    
    // 清理资源
    //vpDisplay::flush(I);
    //vpDisplay::close(I);
    
    if (plotter != nullptr) {
        //delete plotter;
    }
    
    return 0;
}
