#include <iostream>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp_ros/vpROSGrabber.h>



//全局显示窗口对象
//vpDisplayX* display = nullptr;
//vpImage<unsigned char> I;

/***
//用于在图像上显示轨迹点
void display_point_trajectory(const vpImage<unsigned char> &I,const std::vector<vpImagePoint> &vip,
                              std::vector<vpImagePoint *traj_vip>)
{
  for(size_t i=0;i<vip.size();i++){
    if(traj_vip[i].size()){
      //Add the point only if distance with the previous > 1 pixel
      if(vpImagePoint::distance(vip[i].traj_vip[i].back())> 1.){
        traj_vip[i].push_back(vip[i]);
      }
    }
    else{
      traj_vip[i].push_back(vip[i]);
    }
  }
  for(size_t i=0;i<vip.size();i++){
    for(size_t j=1;j<traj_vip[i].size();j++){
      vpDisplay::displayLine(I,traj_vip[i][j-1],traj_vip[i][j],vpColor::green,2);
    }
  }

}
***/

//回调函数处理图像
void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
    //将图像格式转换为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
    //将OpenCV图像转换为ViSP图像
    vpImage<unsigned char> I;
    vpImageConvert::convert(cv_ptr->image,I);
    //创建AprilTag检测器
    vpDetectorAprilTag detector(vpDetectorAprilTag::TAG_36h11);
    //配置检测器
    //设置降采样因子
    detector.setAprilTagQuadDecimate(2.0);
    detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
    //初始化相机参数(这里使用默认数值)
    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(800,800,I.getWidth()/2,I.getHeight()/2);


    //进行AprilTag标签检测
    std::vector<vpHomogeneousMatrix> tagPose;
    double tagSize = 0.01;
    detector.detect(I,tagSize,cam,tagPose);

    //显示图像
    vpDisplayX display(I);

    //遍历检测到的AprilTag标签
    for(size_t i=0;i<detector.getNbObjects();i++){
      //获取标签的ID
      int tagId = detector.getTagsId()[i];
      std::cout<<"Detected AprilTag ID:"<<tagId<<std::endl;
      
      //获取角点坐标(图像坐标系)
      std::vector<vpImagePoint> corners = detector.getPolygon(i);
      for(size_t j=0;j<corners.size();j++){
        std::cout<<"Corner "<<j<<":"<<corners[j]<<std::endl;
        //在图像上绘制角点
        vpDisplay::displayCross(I,corners[j],10,vpColor::red);
      }
      //绘制标签的外接矩形
      vpDisplay::displayPolygon(I,corners,vpColor::green,2);
      vpDisplay::displayText(I,corners[0],std::to_string(tagId),vpColor::green);
  }
    //更新显示
    vpDisplay::flush(I);
  }
  catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception:%s",e.what());
  }

}

int main(int argc,char** argv){
  //初始化ROS节点
  ros::init(argc,argv,"april_tag_detection_node");
  ros::NodeHandle nh;
  //初始化显示窗口
  //I.resize(480,640);
  //display = new vpDisplayX(I);
  //vpDisplay::setTitle(I,"AprilTag detection");
  //订阅/camera/img_raw话题
  ros::Subscriber image_sub = nh.subscribe("/camera/image_raw",1,imageCallback);
  //等待循环回调
  ros::spin();
  return 0;
}



/***
int main(int argc,char **argv)
{
  double opt_tagSize = 0.120;
  std::string opt_robot_ip = "192.168.0.100"
  std::string opt_eMc_filename = "";
  bool display_tag = true;
  int opt_quad_decimate = 2;
  bool opt_verbose = false;
  bool opt_plot = false;
  bool opt_adaptive_gain = false;
  bool opt_task_sequencing = false;
  double convergence_th

}



int main(int argc,char** argv){
  ros::init(argc,argv,"my_visp_example");
  ros::NodeHandle nh;
  
  //创建彩色图像容器
  vpImage<vpRGBa> I;
  //创建ROS抓取器件
  vpROSGrabber g;
  //设置图像话题
  g.setImageTopic("/camera/image_raw");
  //打开抓取器
  g.open(I);
  
  //创建图像显示对象
  vpDisplayX d(I);
  while (ros::ok()){
    //从话题获取图像
    g.acquire(I);
    //显示图像
    vpDisplay:: display(I);
    //刷新显示
    vpDisplay:: flush(I);
    //检测点击
    if(vpDisplay:: getClick(I,false))
      //点击退出
      break;
  }
  return 0;
}
***/
