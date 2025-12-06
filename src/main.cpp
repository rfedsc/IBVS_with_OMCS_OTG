#include <ros/ros.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp_ros/vpROSGrabber.h>

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
