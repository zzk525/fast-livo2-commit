/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.
本文件是FAST-LIVO2的一部分：快速、直接的激光-惯性-视觉里程计

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"  // IMU数据处理
#include "vio.h"              // 视觉惯性里程计
#include "preprocess.h"       // 点云预处理
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <vikit/camera_loader.h>

/**
 * @brief LIV建图器类 - FAST-LIVO2的核心类
 * 
 * 该类整合了激光雷达(LiDAR)、惯性测量单元(IMU)和视觉(Visual)三种传感器，
 * 实现紧耦合的SLAM系统，用于实时位姿估计和地图构建。
 */
class LIVMapper
{
public:
  // ========== 构造与析构 ==========
  LIVMapper(ros::NodeHandle &nh);   // 构造函数
  ~LIVMapper();                      // 析构函数
  
  // ========== 初始化函数 ==========
  void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);  // 初始化ROS订阅者和发布者
  void initializeComponents();       // 初始化各个组件（IMU处理器、VIO管理器等）
  void initializeFiles();            // 初始化输出文件
  
  // ========== 主要运行函数 ==========
  void run();                        // 主循环函数
  void gravityAlignment();           // 重力对齐
  void handleFirstFrame();           // 处理第一帧数据
  void stateEstimationAndMapping();  // 状态估计和建图
  void handleVIO();                  // 处理视觉惯性里程计
  void handleLIO();                  // 处理激光惯性里程计
  void savePCD();                    // 保存点云数据
  void processImu();                 // 处理IMU数据
  
  // ========== 数据同步与传播 ==========
  bool sync_packages(LidarMeasureGroup &meas);  // 同步传感器数据包
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);  // IMU单次传播
  void imu_prop_callback(const ros::TimerEvent &e);  // IMU传播回调函数
  
  // ========== 坐标变换函数 ==========
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);  // 激光雷达坐标变换
  void pointBodyToWorld(const PointType &pi, PointType &po);  // 点从机体坐标系到世界坐标系
  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);  // RGB点从激光雷达坐标系到IMU坐标系
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);  // RGB点从机体坐标系到世界坐标系
  
  // ========== 传感器数据回调函数 ==========
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);  // 标准点云回调
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);  // Livox激光雷达点云回调
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);  // IMU数据回调
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);  // 图像数据回调
  
  // ========== 发布函数 ==========
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);  // 发布RGB图像
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);  // 发布世界坐标系下的帧
  void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);  // 发布视觉子地图
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);  // 发布有效点云
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);  // 发布里程计信息
  void publish_mavros(const ros::Publisher &mavros_pose_publisher);  // 发布MAVROS位姿
  void publish_path(const ros::Publisher pubPath);  // 发布路径
  
  // ========== 工具函数 ==========
  void readParameters(ros::NodeHandle &nh);  // 读取ROS参数
  template <typename T> void set_posestamp(T &out);  // 设置位姿时间戳
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);  // 模板版本：点从机体到世界
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);  // 模板版本：点从机体到世界（返回值）
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);  // 从ROS消息获取OpenCV图像

  // ========== 线程同步 ==========
  std::mutex mtx_buffer, mtx_buffer_imu_prop;  // 互斥锁：数据缓冲区和IMU传播
  std::condition_variable sig_buffer;  // 条件变量：用于线程同步

  // ========== SLAM模式与地图 ==========
  SLAM_MODE slam_mode_;  // SLAM运行模式
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;  // 体素地图：使用八叉树存储
  
  // ========== 路径与话题配置 ==========
  string root_dir;  // 根目录路径
  string lid_topic, imu_topic, seq_name, img_topic;  // 激光雷达话题、IMU话题、序列名称、图像话题
  V3D extT;  // 外参平移向量
  M3D extR;  // 外参旋转矩阵

  // ========== 算法参数 ==========
  int feats_down_size = 0, max_iterations = 0;  // 降采样特征点数量、最大迭代次数

  // ========== 协方差与阈值参数 ==========
  double res_mean_last = 0.05;  // 上一次残差均值
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;  // 陀螺仪协方差、加速度计协方差、逆曝光协方差
  double blind_rgb_points = 0.0;  // RGB点盲区距离
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;  // 上一次时间戳：激光、IMU、图像
  double filter_size_surf_min = 0;  // 表面滤波器最小尺寸
  double filter_size_pcd = 0;  // 点云滤波器尺寸
  double _first_lidar_time = 0.0;  // 第一帧激光雷达时间
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;  // 匹配时间、求解时间、求解常数H时间

  // ========== 功能开关标志 ==========
  bool lidar_map_inited = false, pcd_save_en = false, img_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  // 激光地图已初始化、点云保存使能、图像保存使能、发布有效点使能、位姿输出使能、ROS驱动修复使能、HILTI数据集使能
  int img_save_interval = 1, pcd_save_interval = -1, pcd_save_type = 0;  // 图像保存间隔、点云保存间隔、点云保存类型
  int pub_scan_num = 1;  // 发布扫描数量

  // ========== 状态组 ==========
  StatesGroup imu_propagate, latest_ekf_state;  // IMU传播状态、最新EKF状态

  // ========== IMU传播相关 ==========
  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  // 新IMU数据标志、状态更新标志、IMU传播使能、EKF完成一次标志
  deque<sensor_msgs::Imu> prop_imu_buffer;  // IMU传播缓冲区
  sensor_msgs::Imu newest_imu;  // 最新IMU数据
  double latest_ekf_time;  // 最新EKF时间
  nav_msgs::Odometry imu_prop_odom;  // IMU传播里程计
  ros::Publisher pubImuPropOdom;  // IMU传播里程计发布者
  double imu_time_offset = 0.0;  // IMU时间偏移
  double lidar_time_offset = 0.0;  // 激光雷达时间偏移

  // ========== 重力对齐 ==========
  bool gravity_align_en = false, gravity_align_finished = false;  // 重力对齐使能、重力对齐完成标志

  // ========== 同步标志 ==========
  bool sync_jump_flag = false;  // 同步跳跃标志

  // ========== 传感器使能与标志 ==========
  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  // 激光雷达已推送、IMU使能、重力估计使能、重置标志、偏置估计使能
  bool dense_map_en = false;  // 稠密地图使能
  int img_en = 1, imu_int_frame = 3;  // 图像使能、IMU积分帧数
  bool normal_en = true;  // 法向量使能
  bool exposure_estimate_en = false;  // 曝光估计使能
  double exposure_time_init = 0.0;  // 初始曝光时间
  bool inverse_composition_en = false;  // 逆向组合使能
  bool raycast_en = false;  // 光线投射使能
  int lidar_en = 1;  // 激光雷达使能
  bool is_first_frame = false;  // 是否为第一帧
  
  // ========== 视觉参数 ==========
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  // 网格大小、块大小、网格宽度数量、网格高度数量、块金字塔层级
  double outlier_threshold;  // 外点阈值
  double plot_time;  // 绘图时间
  int frame_cnt;  // 帧计数
  double img_time_offset = 0.0;  // 图像时间偏移
  
  // ========== 数据缓冲区 ==========
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;  // 激光雷达原始数据缓冲区
  deque<double> lid_header_time_buffer;  // 激光雷达时间戳缓冲区
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;  // IMU数据缓冲区
  deque<cv::Mat> img_buffer;  // 图像数据缓冲区
  deque<double> img_time_buffer;  // 图像时间戳缓冲区
  vector<pointWithVar> _pv_list;  // 带方差的点列表
  
  // ========== 外参向量 ==========
  vector<double> extrinT;  // 外参平移向量
  vector<double> extrinR;  // 外参旋转向量
  vector<double> cameraextrinT;  // 相机外参平移向量
  vector<double> cameraextrinR;  // 相机外参旋转向量
  double IMG_POINT_COV;  // 图像点协方差

  // ========== 点云指针 ==========
  PointCloudXYZI::Ptr visual_sub_map;  // 视觉子地图点云
  PointCloudXYZI::Ptr feats_undistort;  // 去畸变后的特征点云
  PointCloudXYZI::Ptr feats_down_body;  // 机体坐标系下的降采样特征点云
  PointCloudXYZI::Ptr feats_down_world;  // 世界坐标系下的降采样特征点云
  PointCloudXYZI::Ptr pcl_w_wait_pub;  // 等待发布的世界坐标系点云
  PointCloudXYZI::Ptr pcl_wait_pub;  // 等待发布的点云
  PointCloudXYZRGB::Ptr pcl_wait_save;  // 等待保存的RGB点云
  PointCloudXYZI::Ptr pcl_wait_save_intensity;  // 等待保存的强度点云

  // ========== 文件输出流 ==========
  ofstream fout_pre, fout_out, fout_visual_pos, fout_lidar_pos, fout_points;
  // 预处理输出、结果输出、视觉位置输出、激光位置输出、点输出

  // ========== PCL滤波器 ==========
  pcl::VoxelGrid<PointType> downSizeFilterSurf;  // 表面降采样滤波器

  // ========== 姿态相关 ==========
  V3D euler_cur;  // 当前欧拉角

  // ========== 测量与状态 ==========
  LidarMeasureGroup LidarMeasures;  // 激光雷达测量组
  StatesGroup _state;  // 当前状态
  StatesGroup  state_propagat;  // 传播状态

  // ========== ROS消息 ==========
  nav_msgs::Path path;  // 路径消息
  nav_msgs::Odometry odomAftMapped;  // 建图后的里程计消息
  geometry_msgs::Quaternion geoQuat;  // 四元数消息
  geometry_msgs::PoseStamped msg_body_pose;  // 机体位姿消息

  // ========== 处理器指针 ==========
  PreprocessPtr p_pre;  // 预处理器指针
  ImuProcessPtr p_imu;  // IMU处理器指针
  VoxelMapManagerPtr voxelmap_manager;  // 体素地图管理器指针
  VIOManagerPtr vio_manager;  // VIO管理器指针

  // ========== ROS发布者与订阅者 ==========
  ros::Publisher plane_pub;  // 平面发布者
  ros::Publisher voxel_pub;  // 体素发布者
  ros::Subscriber sub_pcl;  // 点云订阅者
  ros::Subscriber sub_imu;  // IMU订阅者
  ros::Subscriber sub_img;  // 图像订阅者
  ros::Publisher pubLaserCloudFullRes;  // 全分辨率激光点云发布者
  ros::Publisher pubNormal;  // 法向量发布者
  ros::Publisher pubSubVisualMap;  // 视觉子地图发布者
  ros::Publisher pubLaserCloudEffect;  // 有效激光点云发布者
  ros::Publisher pubLaserCloudMap;  // 激光点云地图发布者
  ros::Publisher pubOdomAftMapped;  // 建图后里程计发布者
  ros::Publisher pubPath;  // 路径发布者
  ros::Publisher pubLaserCloudDyn;  // 动态激光点云发布者
  ros::Publisher pubLaserCloudDynRmed;  // 动态激光点云（去除后）发布者
  ros::Publisher pubLaserCloudDynDbg;  // 动态激光点云调试发布者
  image_transport::Publisher pubImage;  // 图像发布者
  ros::Publisher mavros_pose_publisher;  // MAVROS位姿发布者
  ros::Timer imu_prop_timer;  // IMU传播定时器

  // ========== 统计信息 ==========
  int frame_num = 0;  // 帧数
  double aver_time_consu = 0;  // 平均时间消耗
  double aver_time_icp = 0;  // 平均ICP时间
  double aver_time_map_inre = 0;  // 平均地图增量时间
  bool colmap_output_en = false;  // COLMAP输出使能
};
#endif
