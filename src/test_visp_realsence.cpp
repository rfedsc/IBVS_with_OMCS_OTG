/*#include <ros/ros.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "realsense_visp_example");
    ros::NodeHandle nh;

    // 创建 RealSense 2 相机对象
    vpRealSense2 realsense;

    // 初始化相机
    if (!realsense.open(vpRealSense2::COLOR)) {
        ROS_ERROR("Failed to open RealSense camera");
        return -1;
    }

    // 获取相机图像尺寸 (可以使用相机获取的分辨率)
    unsigned int width = 640;  // 根据实际情况修改
    unsigned int height = 480; // 根据实际情况修改

    // 初始化彩色图像
    vpImage<unsigned char> I(height, width);

    // 创建显示窗口
    vpDisplayX display;
    display.init(I, 100, 100, "RealSense Image");

    // ROS 事件循环
    ros::Rate loop_rate(30);  // 设置循环频率为30Hz
    while (ros::ok()) {
        // 获取图像并显示
        realsense.acquire(I);  // 获取图像数据
        display.display(I);    // 显示图像
        display.flush(I);      // 刷新显示

        // ROS 事件处理
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 关闭相机
    realsense.close();

    return 0;
}
*/
int main(){
    return 0;
}
