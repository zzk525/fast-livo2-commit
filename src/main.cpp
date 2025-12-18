#include "LIVMapper.h" // 包含LIVMapper类的头文件，该类是激光SLAM算法的核心实现

/**
 * @brief 激光SLAM节点的主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出状态码
 *
 * 本函数是整个激光映射系统的入口点，负责初始化ROS节点并启动激光SLAM算法的主流程。
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  // 创建图像传输对象(it)，用于处理图像消息的发布和订阅
  // 这是ROS中专门用于高效处理图像数据的工具
  image_transport::ImageTransport it(nh);

  // 主要用于实现LiDAR-惯性-视觉里程计的建图功能
  LIVMapper mapper(nh);

  // 初始化传感器数据订阅者和结果发布者
  mapper.initializeSubscribersAndPublishers(nh, it);

  ROS_INFO("Waiting for subscribers to be ready...");
  ros::Duration(1).sleep();  // 等待500ms让订阅者完全建立连接
  ros::spinOnce();             // 处理一次回调，确保连接建立
  ROS_INFO("Subscribers ready, starting main loop...");

  // 启动激光SLAM算法的主循环
  // 该方法会进入持续运行状态，不断接收传感器数据，进行位姿估计和地图构建
  mapper.run();

  return 0;
}