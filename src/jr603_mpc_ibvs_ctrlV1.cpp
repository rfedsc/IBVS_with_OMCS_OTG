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
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
//用于处理图像视觉特征,如图像点特征
//用于AprilTag标签检测
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <std_msgs/Float32MultiArray.h>
#include <lys_visp_demo/PixelCoordinates.h>
#include <deque>
#include <numeric>
#include <cmath>


int main(int argc,char **argv){

    //初始化ROS节点
    ros::init(argc,argv,"jr603_ibvs_ctrl");
    ros::NodeHandle nh;
    //发布期望的相机速度
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/camera_velocity",10);
    //发布期望的位置消息
    ros::Publisher feature_pub = nh.advertise<geometry_msgs::Point>("/pd_publish",10);
    //发布期望的图像坐标消息
    ros::Publisher pixel_pub   = nh.advertise<lys_visp_demo::PixelCoordinates>("/image_pixel",10);
    //发布误差矩阵消息
    ros::Publisher error_pub = nh.advertise<std_msgs::Float32MultiArray>("/error_vector",10);

    geometry_msgs::Twist twist_msg;
    //用于发布期望的位置消息
    geometry_msgs::Point point_msg;
    //用于发布期望的图像坐标
    geometry_msgs::Point pixel_msg;
    lys_visp_demo::PixelCoordinates pixel4_msg;
    pixel4_msg.pixel.clear();


    //设置参数
    double opt_tagSize = 0.0815;
    bool display_tag = true;
    int opt_quad_decimate = 2;
    bool opt_adaptive_gain = false;
    bool opt_task_sequencing = true;
    double convergence_threshold = 0.001;
    bool opt_plot = true;
    

    //定义期望的像素坐标
    vpImagePoint ip;
    
    vpHomogeneousMatrix eMc({-0.9988679632, -0.0266672272, -0.0393910032, 32.6903893873/1000,
                             0.0271749617, -0.999553647,  -0.0124108139, 67.2805551257/1000,
                             -0.0390424589, -0.0134672134,  0.9991467963, 27.4906771353/1000,
                            0,0,0,1});



    //设置相机内参
    vpCameraParameters cam(600.750,601.000,325.882,250.568);
   // vpCameraParameters cam(902.812,903.146,647.577,378.309);
    vpImage<unsigned char> I(480,640);
    vpDisplayX display;
    display.init(I,0,0,"Image");
    
    //定义使用标签的类型为TAG_36h11
    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setDisplayTag(display_tag);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);
    
    //定义齐次变换矩阵
    vpHomogeneousMatrix cdMo(vpTranslationVector(0,0,opt_tagSize*6),
                             vpRotationMatrix({1,0,0,0,-1,0,0,0,-1}));
   

    vpHomogeneousMatrix oMo;
    std::vector<vpFeaturePoint> p(4),pd(4);
    std::vector<vpPoint> point(4);
    point[0].setWorldCoordinates(-opt_tagSize/2,-opt_tagSize/2,0);//左下角
    point[1].setWorldCoordinates(opt_tagSize/2,-opt_tagSize/2,0);//右下角
    point[2].setWorldCoordinates(opt_tagSize/2,opt_tagSize/2,0);//右上角
    point[3].setWorldCoordinates(-opt_tagSize/2,opt_tagSize/2,0);//左上角
    
    vpServo task;

    for(size_t i=0;i<p.size();i++){
        task.addFeature(p[i],pd[i]);
    }
    
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::MEAN);

    if(opt_adaptive_gain){
        vpAdaptiveGain lambda(1.5,0.4,30);
        task.setLambda(lambda);
    }
    else{
        task.setLambda(2);
    }
    
    bool has_converged = false;
    bool send_velocities = true;
    double t_init_servo = vpTime::measureTimeMs();
    //plotter指向vpPlot对象的指针,初始化为空
    vpPlot *plotter = nullptr;
    //iter_plot用于迭代绘图的变量,初始值为零
    int iter_plot=0;
    if(opt_plot){
        //如果启用,则创建一个vpPlot对象,用于绘制实时曲线
        plotter = new vpPlot(2,static_cast<int>(250*2),500,static_cast<int>(I.getWidth())+80,10,"Real time curves plotter");
        plotter->setTitle(0,"Visual features error");
        plotter->setTitle(1,"Camera velocities");
        
        plotter->initGraph(0,8);
        plotter->initGraph(1,6);
        //setLegened为每一条曲线添加标签,方便在图表中区分各个变量
        plotter->setLegend(0,0,"error_feat_p1_x");
        plotter->setLegend(0,1,"error_feat_p1_y");
        plotter->setLegend(0,2,"error_feat_p2_x");
        plotter->setLegend(0,3,"error_feat_p2_y");
        plotter->setLegend(0,4,"error_feat_p3_x");
        plotter->setLegend(0,5,"error_feat_p3_y");
        plotter->setLegend(0,6,"error_feat_p4_x");
        plotter->setLegend(0,7,"error_feat_p4_y");
        plotter->setLegend(1,0,"vc_x");
        plotter->setLegend(1,1,"vc_y");
        plotter->setLegend(1,2,"vc_z");
        plotter->setLegend(1,3,"wc_x");
        plotter->setLegend(1,4,"wc_y");
        plotter->setLegend(1,5,"wc_z");

    }
    //存储误差的历史值
    std::deque<double> error_history;
    //设置滑动窗口大小
    const size_t window_size = 100;
    double last_error = 0;

    while(ros::ok()&&!has_converged){

        //获取当前图像
        sensor_msgs::ImageConstPtr img_msg = ros::topic::waitForMessage<sensor_msgs::Image>("camera/color/image_raw",nh);
        if(!img_msg) continue;

        //转换为OpenCV图像格式        
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::RGB8);
        cv::Mat cv_img = cv_ptr->image;
        vpImageConvert::convert(cv_img,I);
        display.display(I);
        display.flush(I);

        //检测AprilTag标签
        std::vector<vpHomogeneousMatrix> cMo_vec;
        detector.detect(I,opt_tagSize,cam,cMo_vec);

        //获取标签的角点
        std::vector<vpImagePoint> corners = detector.getPolygon(0);

        for(size_t i=0;i<corners.size();i++){
            std::cout<<"Corner i++"<<i+1<<":("
                     <<corners[i].get_u()<<","
                     <<corners[i].get_v()<<")"<<std::endl;
        }
        
        vpColVector v_c(6);
        

        if(cMo_vec.size()==1){

            vpHomogeneousMatrix cMo = cMo_vec[0];
            static bool first_time = true;

            if(first_time){

                std::vector<vpHomogeneousMatrix> v_oMo(2),v_cdMc(2);
                v_oMo[1].buildFrom(0,0,0,0,0,M_PI);
                for(size_t i=0;i<2;i++){
                    //计算两个可能的相对位姿
                    v_cdMc[i] = cdMo*v_oMo[i]*cMo.inverse();
                    ROS_INFO_STREAM("cdMc["<<i<<"]"<<v_cdMc[i]);
                }
                //选择正确的旋转姿态
                if(std::fabs(v_cdMc[0].getThetaUVector().getTheta())<std::fabs(v_cdMc[1].getThetaUVector().getTheta())){
                    oMo = v_oMo[0];
                    ROS_INFO_STREAM("oMo[0]"<<oMo);
                }
                else{
                    oMo = v_oMo[1];
                    ROS_INFO_STREAM("oMo[1]"<<oMo);
                }
            }
            ROS_INFO_STREAM("oMo:\n"<<oMo);
            ROS_INFO_STREAM("cMo:\n"<<cMo);
            //计算视觉特征点
            for(size_t i=0;i<point.size();i++){
                //cP在相机坐标系中的坐标
                //p_存储经过投影后的二维坐标
                vpColVector cP, p_;

                point[i].changeFrame(cdMo*oMo,cP);
               //point[i].changeFrame(cdMo,cP);
                //将三维点投影到二维平面
                point[i].projection(cP,p_);
                pd[i].set_x(p_[0]);
                pd[i].set_y(p_[1]);
                pd[i].set_Z(cP[2]);
                //ip存储像素坐标
                vpMeterPixelConversion::convertPoint(cam,pd[i].get_x(),pd[i].get_y(),ip);

                //输出期望的像素坐标
                ROS_INFO_STREAM("ip is"<<ip);
                float u = static_cast<float>(ip.get_u());
                float v = static_cast<float>(ip.get_v());
                pixel4_msg.pixel.push_back(u);
                pixel4_msg.pixel.push_back(v);
                    

                //输出发布期望位置pd[i]
                point_msg.x = pd[i].get_x();
                point_msg.y = pd[i].get_y();
                point_msg.z = pd[i].get_Z();
                //发布每个期望位置
                feature_pub.publish(point_msg);
                ROS_INFO_STREAM("cdMo:\r\n"<<cdMo);
                //输出p_[i]的二维投影坐标
                ROS_INFO_STREAM("p_["<<i<<"] is:"<<p_[i]);
                //输入pd[i]的坐标和深度
                ROS_INFO_STREAM("pd["<<i<<"]:x="<<pd[i].get_x()<<",y="<<pd[i].get_y()<<",Z="<<pd[i].get_Z());
           }
    
           if(pixel4_msg.pixel.size()==8){
                //发布消息
                pixel_pub.publish(pixel4_msg);
                ROS_INFO_STREAM("pixel4_msg is"<<pixel4_msg);
                //清空pixel数组,准备下一次循环
                pixel4_msg.pixel.clear();
           }
    
            //更新视觉特征
            for(size_t i=0;i<corners.size();i++){
                vpFeatureBuilder::create(p[i],cam,corners[i]);
                vpColVector cp;
                point[i].changeFrame(cMo,cp);
                p[i].set_Z(cp[2]);
                //输出p[i]的坐标和深度
                ROS_INFO_STREAM("p["<<i<<"]:x="<<p[i].get_x()<<",y="<<p[i].get_y()<<",Z="<<p[i].get_Z());
            }
            
            double lambda = 1.0;
            task.computeError();				//计算特征误差
            vpColVector e = task.getError(); 			//获取特征误差
            vpMatrix L = task.computeInteractionMatrix();	//计算交互矩阵
            vpMatrix L_pinv = L.pseudoInverse();		//计算伪逆
            v_c = -lambda*L_pinv*e;				//计算相机速度
                    
            //v_c = opt_task_sequencing?task.computeControlLaw((vpTime::measureTimeMs()-t_init_servo)/1000.0)
            //                     :task.computeControlLaw();
                
            double error = task.getError().sumSquare();

            vpColVector Error = task.getError();
            //用于发布误差向量
            std_msgs::Float32MultiArray error_msg;
            error_msg.data.clear();
            for(int i=0;i<Error.size();i++){
                error_msg.data.push_back(Error[i]);
            }
            error_pub.publish(error_msg);
            ROS_INFO_STREAM("Current error sumSquare :"<<error);
            ROS_INFO_STREAM("error_msg published...");
            ROS_INFO_STREAM("ErrorVector is:\r\n"<<Error);

            if(first_time){
                first_time = false;
            }
            
            //可视化速度和误差
            if(opt_plot){
                plotter->plot(0,iter_plot,task.getError());
                plotter->plot(1,iter_plot,v_c);
                iter_plot++;
            }

            if(error<convergence_threshold) has_converged = true;
            if(!send_velocities){
                v_c = 0;
            }

            twist_msg.linear.x  = v_c[0];
            twist_msg.linear.y  = v_c[1];
            twist_msg.linear.z  = v_c[2];
            twist_msg.angular.x = v_c[3];
            twist_msg.angular.y = v_c[4];
            twist_msg.angular.z = v_c[5];
        
            if(send_velocities){
                //发布相机速度
                vel_pub.publish(twist_msg);
                ROS_INFO_STREAM("Published velocity:["
                                <<std::fixed<<std::setprecision(6)
                                <<twist_msg.linear.x<<","<<twist_msg.linear.y<<","
                                <<twist_msg.linear.z<<","<<twist_msg.angular.x<<","
                                <<twist_msg.angular.y<<","<<twist_msg.angular.z<<"]");
            
            }

            vpDisplay::displayText(I,20,20,"Press 'q' to quit",vpColor::red);
            if(vpDisplay::getClick(I,false)) break;
    
        }
    }
    vpDisplay::flush(I);
    ros::spinOnce();
    
    vpDisplay::close(I);
    return 0;
}
