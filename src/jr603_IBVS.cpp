/*#include <ros/ros.h>
#include <iostream>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
//用于AprilTag标签检测
#include <visp3/detection/vpDetectorAprilTag.h>
//用于显示图像的GDI和X窗口系统
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
//用于绘制图形或曲线
#include <visp3/gui/vpPlot.h>
//用于图像输入输出操作
#include <visp3/io/vpImageIo.h>
//用于UR机械臂的控制
//#include <visp3/robot/vpRobotUniversalRobots.h>
//用于Intel RealSense相机的管理
#include <visp3/sensor/vpRealSense2.h>
#include <librealsense2/rs.hpp>
//用于处理图像视觉特征,如图像点特征
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
//用于视觉伺服控制以及显示
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <geometry_msgs/Twist.h>

 
#if defined(VISP_HAVE_REALSENSE2) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_UR_RTDE)
//命名空间管理
#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
#endif

//用于在图像上显示轨迹点 
void display_point_trajectory(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip,
                              std::vector<vpImagePoint> *traj_vip)
{
  for (size_t i = 0; i < vip.size(); i++) {
    if (traj_vip[i].size()) {
      // Add the point only if distance with the previous > 1 pixel
      if (vpImagePoint::distance(vip[i], traj_vip[i].back()) > 1.) {
        traj_vip[i].push_back(vip[i]);
      }
    }
    else {
      traj_vip[i].push_back(vip[i]);
    }
  }
  for (size_t i = 0; i < vip.size(); i++) {
    for (size_t j = 1; j < traj_vip[i].size(); j++) {
      vpDisplay::displayLine(I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2);
    }
  }
}


int main(int argc,char **argv){
    //ROS节点初始化
    ros::init(argc,argv,"ibvs_apriltag");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/camera_velocity",10);
    geometry_msgs::Twist twist_msg;


    double opt_tagSize = 0.120;
    bool display_tag = true;
    bool opt_adaptive_gain = false;
    bool opt_plot = false;
    bool opt_task_sequencing = false;
    bool opt_verbose = false;
    int opt_quad_decimate = 2;

	//初始化相机
	vpRealSense2 rs;
	rs2::config config;
	unsigned int width = 640, height = 480;
	config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
	config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
	rs.open(config);
	
	//设置相机外参
	vpPoseVector ePc;
	ePc[0] = -0.0312543;
	ePc[1] = -0.0584638;
	ePc[2] = 0.0309834;
	ePc[3] = -0.00506562;
	ePc[4] = -0.00293325;
	ePc[5] = 0.0117901;
	//相机外参齐次变换矩阵
	vpHomogeneousMatrix eMc(ePc);
	//设置相机内参
	vpCameraParameters cam = rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion);
	
	vpImage<unsigned char> I(height, width);
	
	#if defined(VISP_HAVE_X11)
		vpDisplayX dc(I, 10, 10, "Color image");
	#elif defined(VISP_HAVE_GDI)
		vpDisplayGDI dc(I, 10, 10, "Color image");
	#endif
	
	//定义使用的AprilTag类型为TAG_36h11
	vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
	//选择姿态估计的方法为HOMOGRAPHY_VIRTUAL_VS(虚拟视图单应性)
	vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
	// vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
	
	vpDetectorAprilTag detector(tagFamily);
	detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
	//是否在检测时显示Apriltag标签的边框,id等信息
	detector.setDisplayTag(display_tag);
    //这行代码用于设置检测图像时的下采样因子，即对图像进行缩小以加快检测速度
	detector.setAprilTagQuadDecimate(opt_quad_decimate);
    //定义多个齐次变换矩阵,用于表示不同坐标系之间的变换关系
    vpHomogeneousMatrix cdMc,cMo,oMo;
    
	//设置期望姿态的变换矩阵cdMo,其位置在相机坐标系中沿着
	//Z轴方向(即向前)移动opt_tagSize*3,并进行相应旋转
    vpHomogeneousMatrix cdMo(vpTranslationVector(0, 0, opt_tagSize * 3),
                             vpRotationMatrix({ 1, 0, 0, 0, -1, 0, 0, 0, -1 }));

    //创建视觉特征点
    std::vector<vpFeaturePoint> p(4),pd(4);
    
    //定义4个与AprilTag对应的3D点
    std::vector<vpPoint> point(4);
    point[0].setWorldCoordinates(-opt_tagSize/2.,-opt_tagSize/2.,0);
    point[1].setWorldCoordinates( opt_tagSize/2.,-opt_tagSize/2.,0);
    point[2].setWorldCoordinates( opt_tagSize/2., opt_tagSize/2.,0);
    point[3].setWorldCoordinates(-opt_tagSize/2., opt_tagSize/2.,0);

    vpServo task;
    //将当前特征点添加到伺服控制任务中
    for (size_t i = 0; i < p.size(); i++) {
      task.addFeature(p[i], pd[i]);
    }

    //设置伺服模式
    task.setServo(vpServo::EYEINHAND_CAMERA);
    //设置交互矩阵类型
    task.setInteractionMatrixType(vpServo::CURRENT);
	//设置增益
	//如果设置了自适应增益,则使用vpAdaptiveGain lambda来动态调整增益
    if (opt_adaptive_gain) {
      vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
      task.setLambda(lambda);
    }
	//如果未启用自适应增益,则使用固定增益0.5,则控制器响应速度恒定
    else {
      task.setLambda(0.5);
    }


	//plotter指向vpPlot对象的指针,初始化为空
    vpPlot *plotter = nullptr;
	//iter_plot用于迭代的绘图变量,初始值为零
    int iter_plot = 0;
	//判断是否启用绘图功能
    if (opt_plot) {
	  //如果启用,则创建一个vpPlot对象,用于绘制实时曲线
      plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10,
                           "Real time curves plotter");
      plotter->setTitle(0, "Visual features error");
      plotter->setTitle(1, "Camera velocities");
      //初始化图表和设置图例
      //初始化第一个图表(视觉特征误差)的8条曲线,分别对应4个视觉特征点的x和y坐标误差
      plotter->initGraph(0, 8);
      //初始化第二个图表(相机速度)的6条曲线,分别对应3个平移速度和三个旋转速度
      plotter->initGraph(1, 6);
      //setLegend为每一条曲线添加标签,方便在图表中区分各个变量
      plotter->setLegend(0, 0, "error_feat_p1_x");
      plotter->setLegend(0, 1, "error_feat_p1_y");
      plotter->setLegend(0, 2, "error_feat_p2_x");
      plotter->setLegend(0, 3, "error_feat_p2_y");
      plotter->setLegend(0, 4, "error_feat_p3_x");
      plotter->setLegend(0, 5, "error_feat_p3_y");
      plotter->setLegend(0, 6, "error_feat_p4_x");
      plotter->setLegend(0, 7, "error_feat_p4_y");
      plotter->setLegend(1, 0, "vc_x");
      plotter->setLegend(1, 1, "vc_y");
      plotter->setLegend(1, 2, "vc_z");
      plotter->setLegend(1, 3, "wc_x");
      plotter->setLegend(1, 4, "wc_y");
      plotter->setLegend(1, 5, "wc_z");
    }

    //设置控制相关的布尔变量
    bool final_quit = false;
    //表示伺服任务是否已经收敛
    bool has_converged = false;
    //表示是否发送速度命令给机器人
    bool send_velocites = false;
    //表示伺服任务是否已经启动
    bool servo_started = false;
    //用于记录视觉特征点在图像中的轨迹
    std::vector<vpImagePoint> *traj_corners = nullptr;

    //计时与机器人控制设置
    //t_init_servo记录了伺服任务开始的时间,用于计算任务的时长与控制的节奏
    static double t_init_servo = vpTime::measureTimeMs();

    while(!has_converged && ! final_quit){
        //获取当前时间
        double t_start = vpTime::measureTimeMs();
        //从相机(或其他图像源)获取一帧图像,并将其存储在图像对象I中
        rs.acquire(I);
        //显示图像
        vpDisplay::display(I);
        //视觉检测标签,检测结果保存在cMo_vec中
        std::vector<vpHomogeneousMatrix> cMo_vec;
        detector.detect(I,opt_tagSize,cam,cMo_vec);

    //相机速度向量初始化
    vpColVector v_c(6);
    
    //如果检测到一个标签
    if(cMo_vec.size()==1){
        cMo = cMo_vec[0];

        static bool first_time = true;
        if (first_time) {
		  //处理旋转问题,定义两个齐次变换矩阵向量v_oMo,v_cdMc
		  //v_oMo[1]被设置为一个旋转矩阵,在z轴旋转180
          std::vector<vpHomogeneousMatrix> v_oMo(2), v_cdMc(2);
          v_oMo[1].build(0, 0, 0, 0, 0, M_PI);
          for (size_t i = 0; i < 2; i++) {
		  //计算两个可能的相对位姿
            v_cdMc[i] = cdMo * v_oMo[i] * cMo.inverse();
          }
		  //选择正确的旋转姿态
          if (std::fabs(v_cdMc[0].getThetaUVector().getTheta()) < std::fabs(v_cdMc[1].getThetaUVector().getTheta())) {
            oMo = v_oMo[0];
          }
          else {
            std::cout << "为了避免相机发生180度旋转,修改了期望的坐标系" << std::endl;
            oMo = v_oMo[1]; // Introduce PI rotation
          }

          //计算期望特征位置
          for(size_t i=0,i<point.size();i++){
            vpColVector cp,p_;
            point[i].changeFrame(cdMo * oMo, cP);
            point[i].projection(cP, p_);
 
            pd[i].set_x(p_[0]);
            pd[i].set_y(p_[1]);
            pd[i].set_Z(cP[2]);
          }

    }
    //获取标签的角点
    std::vector<vpImagePoint> corners = detector.getPolygon(0);
    //更新视觉特征
    for(size_t i=0;i<corners.size();i++){
        vpFeatureBuilder::create(p[i],cam,corners[i]);
        //计算特征点在相机坐标系下的深度z坐标
        vpColVector cP;
        point[i].changeFrame(cMo,cP);
        p[i].set_Z(cp[2])
    }


    //计算控制率
    //检查是否使用了任务序列化
    if(opt_task_sequencing){
        if(!servo_started){
            if(send_velocities){
                servo_started = true;
            }
            //记录伺服开始的时间,用于计算伺服运行时间
            t_init_servo = vpTime::measureTimeMs();
        }
        //计算控制速度v_c,其中传入的参数是伺服已运行的时间
         v_c = task.computeControlLaw((vpTime::measureTimeMs() - t_init_servo) / 1000.);
    }
    else{
        //直接计算控制速度,不考虑时间
        v_c = task.computeControlLaw();
    }

    //显示当前和期望的特征点
    vpServoDisplay::display(task, cam, I);
    for(size_t i=0;i<corners.size();i++){
        std::stringstream ss;
        ss<<i;
        //显示当前特征点的索引
        vpDisplay::displayText(I, corners[i] + vpImagePoint(15, 15), ss.str(), vpColor::red);
        //显示期望特征点的索引
        vpImagePoint ip;
        vpMeterPixelConversion::convertPoint(cam, pd[i].get_x(), pd[i].get_y(), ip);
        vpDisplay::displayText(I, ip + vpImagePoint(15, 15), ss.str(), vpColor::red);
    }
    //初始化并显示特征点轨迹
    if (first_time) {
        traj_corners = new std::vector<vpImagePoint>[corners.size()];
    }
    
    display_point_trajectory(I, corners, traj_corners);

    if (opt_plot) {
        plotter->plot(0, iter_plot, task.getError());
        plotter->plot(1, iter_plot, v_c);
        iter_plot++;
   }
   //输出调试信息
   if (opt_verbose) {
    std::cout << "v_c: " << v_c.t() << std::endl;
   }
   //计算误差并显示
   double error = task.getError().sumSquare();

   std::stringstream ss;
   ss << "error: " << error;
   vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);
   //如果启用了详细模式,在控制台中输出误差值
   if (opt_verbose)
    std::cout << "error: " << error << std::endl;
	//判断是否收敛
    if (error < convergence_threshold) {
        has_converged = true;
		//将伺服任务已收敛的信息显示在图像上,并输出到控制台中
        std::cout << "Servo task has converged"    << "\n";
        vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
    }
		//初始化标志
    if (first_time) {
        first_time = false;
     }

    //当未检测到标签时,将控制速度设置为零
    if(!send_velocities){
        v_c = 0;
    }

    //发送相机速度
    //ROS消息发布:将速度v_c发送出去
    twist_msg.linear.x = v_c[0];
    twist_msg.linear.x = v_c[1];
    twist_msg.linear.x = v_c[2];
    twist_msg.angular.x = v_c[3];
    twist_msg.angular.y = v_c[4];
    twist_msg.angular.z = v_c[5];
    //发布速度消息
    if(send_velocities){
        vel_pub.publish(twist.msg);
        ROS_INFO_STREAM("Published camera velocity: [" << twist_msg.linear.x << ", "
            << twist_msg.linear.y << ", " << twist_msg.linear.z << ", "
            << twist_msg.angular.x << ", " << twist_msg.angular.y << ", "
            << twist_msg.angular.z << "]");
    
    }

 }

}
*/

int main(){
    return 0;
}

