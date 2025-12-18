/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.
本文件是FAST-LIVO2的一部分：快速、直接的激光-惯性-视觉里程计

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <utils/so3_math.h>  // SO(3)数学库
#include <utils/types.h>      // 类型定义
#include <utils/color.h>      // 颜色工具
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <sophus/se3.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;
using namespace Sophus;

// ========== 宏定义 ==========
#define print_line std::cout << __FILE__ << ", " << __LINE__ << std::endl;  // 打印当前文件和行号
#define G_m_s2 (9.81)   // 重力加速度常数（广东/中国地区）单位：m/s²
#define DIM_STATE (19)  // 状态维度（SO(3)维度设为3）：旋转(3) + 位置(3) + 曝光(1) + 速度(3) + 陀螺仪偏置(3) + 加速度计偏置(3) + 重力(3)
#define INIT_COV (0.01) // 初始协方差值
#define SIZE_LARGE (500)  // 大尺寸常量
#define SIZE_SMALL (100)  // 小尺寸常量
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]  // 从数组提取向量元素
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]  // 从数组提取矩阵元素
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))  // 调试文件目录路径

/**
 * @brief 激光雷达类型枚举
 * 支持多种常见的激光雷达传感器
 */
enum LID_TYPE
{
  AVIA = 1,       // DJI Livox Avia
  VELO16 = 2,     // Velodyne VLP-16
  OUST64 = 3,     // Ouster OS1-64
  L515 = 4,       // Intel RealSense L515
  XT32 = 5,       // Hesai XT32
  PANDAR128 = 6,  // Hesai Pandar128
  ROBOSENSE = 7   // RoboSense系列
};

/**
 * @brief SLAM运行模式枚举
 * 定义系统可以运行的不同模式
 */
enum SLAM_MODE
{
  ONLY_LO = 0,   // 仅激光里程计（Lidar Odometry）
  ONLY_LIO = 1,  // 仅激光惯性里程计（Lidar-Inertial Odometry）
  LIVO = 2       // 激光惯性视觉里程计（Lidar-Inertial-Visual Odometry）
};

/**
 * @brief EKF状态枚举
 * 表示扩展卡尔曼滤波器当前处理的传感器类型
 */
enum EKF_STATE
{
  WAIT = 0,  // 等待状态
  VIO = 1,   // 视觉惯性里程计状态
  LIO = 2,   // 激光惯性里程计状态
  LO = 3     // 激光里程计状态
};

/**
 * @brief 测量数据组结构体
 * 用于在LIVO模式下同步和管理多传感器数据（IMU、LiDAR、相机）
 */
struct MeasureGroup
{
  double vio_time;  // VIO（视觉惯性里程计）处理的目标时间戳，通常是图像捕获时间
                    // 用于视觉特征跟踪和光度误差优化
  
  double lio_time;  // LIO（激光惯性里程计）处理的目标时间戳
                    // 在LIVO模式下，lio_time通常等于vio_time（图像时间）
                    // 用于LiDAR点云的去畸变和状态更新
  
  deque<sensor_msgs::Imu::ConstPtr> imu;  // IMU数据队列
                                           // 存储[last_update_time, lio_time]时间段内的IMU数据
                                           // 用于状态预测和运动补偿
  
  cv::Mat img;  // 图像数据
                // 在VIO阶段使用，用于视觉特征提取和光度BA优化
  
  /**
   * @brief 构造函数，初始化时间戳为0
   */
  MeasureGroup()
  {
    vio_time = 0.0;
    lio_time = 0.0;
  };
};

/**
 * @brief LiDAR测量数据组结构体
 * 用于存储一帧LiDAR扫描的相关数据和状态信息
 */
struct LidarMeasureGroup
{
  double lidar_frame_beg_time;      // LiDAR帧开始时间戳
  double lidar_frame_end_time;      // LiDAR帧结束时间戳
  double last_lio_update_time;      // 上一次LIO更新的时间戳
  PointCloudXYZI::Ptr lidar;        // 原始LiDAR点云数据
  PointCloudXYZI::Ptr pcl_proc_cur; // 当前处理的点云数据
  PointCloudXYZI::Ptr pcl_proc_next;// 下一帧待处理的点云数据
  deque<struct MeasureGroup> measures; // IMU和图像测量数据队列
  EKF_STATE lio_vio_flg;            // EKF状态标志（WAIT/VIO/LIO/LO）
  int lidar_scan_index_now;         // 当前LiDAR扫描索引

  /**
   * @brief 构造函数，初始化所有成员变量
   */
  LidarMeasureGroup()
  {
    lidar_frame_beg_time = -0.0;    // 初始化帧开始时间
    lidar_frame_end_time = 0.0;     // 初始化帧结束时间
    last_lio_update_time = -1.0;    // 初始化上次更新时间为-1（表示未更新）
    lio_vio_flg = WAIT;             // 初始状态设为等待
    this->lidar.reset(new PointCloudXYZI());      // 创建原始点云智能指针
    this->pcl_proc_cur.reset(new PointCloudXYZI()); // 创建当前处理点云智能指针
    this->pcl_proc_next.reset(new PointCloudXYZI());// 创建下一帧点云智能指针
    this->measures.clear();         // 清空测量数据队列
    lidar_scan_index_now = 0;       // 初始化扫描索引为0
    last_lio_update_time = -1.0;    // 再次确保更新时间初始化
  };
};

/**
 * @brief 带方差的点结构体
 * 用于存储点在不同坐标系下的位置及其不确定性信息
 */
typedef struct pointWithVar
{
  Eigen::Vector3d point_b;     // 激光雷达机体坐标系下的点坐标
  Eigen::Vector3d point_i;     // IMU机体坐标系下的点坐标
  Eigen::Vector3d point_w;     // 世界坐标系下的点坐标
  Eigen::Matrix3d var_nostate; // 去除状态协方差后的方差矩阵
  Eigen::Matrix3d body_var;    // 机体坐标系下的方差矩阵
  Eigen::Matrix3d var;         // 完整的方差矩阵
  Eigen::Matrix3d point_crossmat;  // 点的叉乘矩阵（用于李代数运算）
  Eigen::Vector3d normal;      // 点所在平面的法向量
  
  /**
   * @brief 构造函数，初始化所有成员为零
   */
  pointWithVar()
  {
    var_nostate = Eigen::Matrix3d::Zero();
    var = Eigen::Matrix3d::Zero();
    body_var = Eigen::Matrix3d::Zero();
    point_crossmat = Eigen::Matrix3d::Zero();
    point_b = Eigen::Vector3d::Zero();
    point_i = Eigen::Vector3d::Zero();
    point_w = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
  };
} pointWithVar;


/**
 * @brief 状态组结构体
 * 存储ESKF（误差状态卡尔曼滤波器）的完整状态向量和协方差矩阵
 * 状态维度为19：旋转(3) + 位置(3) + 曝光(1) + 速度(3) + 陀螺仪偏置(3) + 加速度计偏置(3) + 重力(3)
 */
struct StatesGroup
{
  /**
   * @brief 默认构造函数
   * 初始化所有状态为零或单位值，协方差矩阵为对角阵
   */
  StatesGroup()
  {
    this->rot_end = M3D::Identity();  // 旋转矩阵初始化为单位阵
    this->pos_end = V3D::Zero();      // 位置初始化为零
    this->vel_end = V3D::Zero();      // 速度初始化为零
    this->bias_g = V3D::Zero();       // 陀螺仪偏置初始化为零
    this->bias_a = V3D::Zero();       // 加速度计偏置初始化为零
    this->gravity = V3D::Zero();      // 重力向量初始化为零
    this->inv_expo_time = 1.0;        // 逆曝光时间初始化为1.0
    this->cov = MD(DIM_STATE, DIM_STATE)::Identity() * INIT_COV;  // 协方差矩阵初始化
    this->cov(6, 6) = 0.00001;        // 曝光时间的协方差设置为较小值
    this->cov.block<9, 9>(10, 10) = MD(9, 9)::Identity() * 0.00001;  // 偏置和重力的协方差设置为较小值
  };

  /**
   * @brief 拷贝构造函数
   * @param b 要拷贝的状态组
   */
  StatesGroup(const StatesGroup &b)
  {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->inv_expo_time = b.inv_expo_time;
    this->cov = b.cov;
  };

  /**
   * @brief 赋值运算符重载
   * @param b 要赋值的状态组
   * @return 当前状态组的引用
   */
  StatesGroup &operator=(const StatesGroup &b)
  {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->inv_expo_time = b.inv_expo_time;
    this->cov = b.cov;
    return *this;
  };

  /**
   * @brief 加法运算符重载（流形上的加法）
   * 在流形上进行状态更新，旋转使用李群的指数映射
   * @param state_add 状态增量向量（19维）
   * @return 更新后的新状态组
   */
  StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add)
  {
    StatesGroup a;
    a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));  // 旋转更新（李群乘法）
    a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);      // 位置更新
    a.inv_expo_time = this->inv_expo_time + state_add(6, 0);      // 曝光时间更新
    a.vel_end = this->vel_end + state_add.block<3, 1>(7, 0);      // 速度更新
    a.bias_g = this->bias_g + state_add.block<3, 1>(10, 0);       // 陀螺仪偏置更新
    a.bias_a = this->bias_a + state_add.block<3, 1>(13, 0);       // 加速度计偏置更新
    a.gravity = this->gravity + state_add.block<3, 1>(16, 0);     // 重力向量更新

    a.cov = this->cov;  // 协方差矩阵保持不变
    return a;
  };

  /**
   * @brief 复合赋值加法运算符重载
   * 在当前状态上直接进行更新
   * @param state_add 状态增量向量（19维）
   * @return 当前状态组的引用
   */
  StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
  {
    this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    this->pos_end += state_add.block<3, 1>(3, 0);
    this->inv_expo_time += state_add(6, 0);
    this->vel_end += state_add.block<3, 1>(7, 0);
    this->bias_g += state_add.block<3, 1>(10, 0);
    this->bias_a += state_add.block<3, 1>(13, 0);
    this->gravity += state_add.block<3, 1>(16, 0);
    return *this;
  };

  /**
   * @brief 减法运算符重载（流形上的减法）
   * 计算两个状态之间的差异，旋转使用李群的对数映射
   * @param b 要减去的状态组
   * @return 状态差异向量（19维）
   */
  Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b)
  {
    Matrix<double, DIM_STATE, 1> a;
    M3D rotd(b.rot_end.transpose() * this->rot_end);  // 计算相对旋转
    a.block<3, 1>(0, 0) = Log(rotd);                   // 旋转差异（李代数）
    a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;   // 位置差异
    a(6, 0) = this->inv_expo_time - b.inv_expo_time;   // 曝光时间差异
    a.block<3, 1>(7, 0) = this->vel_end - b.vel_end;   // 速度差异
    a.block<3, 1>(10, 0) = this->bias_g - b.bias_g;    // 陀螺仪偏置差异
    a.block<3, 1>(13, 0) = this->bias_a - b.bias_a;    // 加速度计偏置差异
    a.block<3, 1>(16, 0) = this->gravity - b.gravity;  // 重力向量差异
    return a;
  };

  /**
   * @brief 重置位姿
   * 将旋转、位置和速度重置为零或单位值
   */
  void resetpose()
  {
    this->rot_end = M3D::Identity();
    this->pos_end = V3D::Zero();
    this->vel_end = V3D::Zero();
  }

  // ========== 状态变量 ==========
  M3D rot_end;                              // 估计的姿态（旋转矩阵），在激光雷达帧末尾时刻
  V3D pos_end;                              // 估计的位置，在激光雷达帧末尾时刻（世界坐标系）
  V3D vel_end;                              // 估计的速度，在激光雷达帧末尾时刻（世界坐标系）
  double inv_expo_time;                     // 估计的逆曝光时间（无尺度）
  V3D bias_g;                               // 陀螺仪偏置
  V3D bias_a;                               // 加速度计偏置
  V3D gravity;                              // 估计的重力加速度向量
  Matrix<double, DIM_STATE, DIM_STATE> cov; // 状态协方差矩阵（19x19）
};

/**
 * @brief 设置6自由度位姿
 * 将Eigen格式的位姿数据转换为Pose6D结构体
 * 
 * @tparam T 数据类型（通常为double或float）
 * @param t 时间偏移
 * @param a 加速度向量（3维）
 * @param g 角速度向量（3维）
 * @param v 速度向量（3维）
 * @param p 位置向量（3维）
 * @param R 旋转矩阵（3x3）
 * @return Pose6D 包含完整位姿信息的结构体
 */
template <typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p,
                const Matrix<T, 3, 3> &R)
{
  Pose6D rot_kp;
  rot_kp.offset_time = t;  // 设置时间偏移
  for (int i = 0; i < 3; i++)
  {
    rot_kp.acc[i] = a(i);  // 设置加速度
    rot_kp.gyr[i] = g(i);  // 设置角速度
    rot_kp.vel[i] = v(i);  // 设置速度
    rot_kp.pos[i] = p(i);  // 设置位置
    for (int j = 0; j < 3; j++)
      rot_kp.rot[i * 3 + j] = R(i, j);  // 设置旋转矩阵（按行存储）
  }
  return move(rot_kp);  // 使用移动语义返回
}

#endif
