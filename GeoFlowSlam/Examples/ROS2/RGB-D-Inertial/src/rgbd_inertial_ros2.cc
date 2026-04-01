/**
 * Modified for Debugging GeoFlowSlam Initialization Issues
 */

#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> // 包含resize等
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <thread>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>

// PCL与稀疏点云发布所需头文件
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "../../../include/MapPoint.h" // 获取稀疏地图点的 3D 坐标

#include "../../../include/ImuTypes.h"
#include "../../../include/System.h"

using namespace std;

float shift = 0;

class ImageGrabber : public rclcpp::Node {
 public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, const bool bRect, const bool bClahe)
      : Node("image_grabber"),
        mpSLAM(pSLAM),
        do_rectify(bRect),
        mbClahe(bClahe) {
    
    // QoS 设置
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort); // 改为 BestEffort 以兼容更多相机发布者
    
    rclcpp::QoS qos_imu(100);
    qos_imu.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    rclcpp::QoS qos_odom(100);
    qos_odom.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    RCLCPP_INFO(this->get_logger(), "正在订阅话题...");
    RCLCPP_INFO(this->get_logger(), "RGB: /camera/camera/color/image_raw");
    RCLCPP_INFO(this->get_logger(), "Depth: /camera/camera/aligned_depth_to_color/image_raw");
    RCLCPP_INFO(this->get_logger(), "IMU: /camera/camera/imu");
    RCLCPP_INFO(this->get_logger(), "Odom: /robot_odom");

    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw", qos_profile,
        std::bind(&ImageGrabber::GrabImageRgb, this, std::placeholders::_1));
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/aligned_depth_to_color/image_raw", qos_profile,
        std::bind(&ImageGrabber::GrabImageDepth, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/camera/camera/imu", qos_imu,
        std::bind(&ImageGrabber::GrabImu, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot_odom", qos_odom,
        std::bind(&ImageGrabber::GrabOdom, this, std::placeholders::_1));
    // 稀疏点云地图发布器
    rclcpp::QoS sparse_qos(rclcpp::KeepLast(1));
    sparse_qos.transient_local(); // 确保 RViz 随时订阅都能获取最新点云
    sparse_qos.reliable();
    sparse_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/sparse_map_points", sparse_qos);
    
    // 创建一个定时器，每 500 毫秒 (2Hz) 发布一次稀疏点云
    sparse_map_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ImageGrabber::PublishSparseMap, this));
  }

  void GrabImageRgb(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mBufMutexRgb);
    if (!imgRgbBuf.empty()) imgRgbBuf.pop();
    imgRgbBuf.push(msg);
  }

  void GrabImageDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mBufMutexDepth);
    if (!imgDepthBuf.empty()) imgDepthBuf.pop();
    imgDepthBuf.push(msg);
  }

  void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    std::lock_guard<std::mutex> lock(mBufMutex);
    imuBuf.push(imu_msg);
  }

  void GrabOdom(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    std::lock_guard<std::mutex> lock(mBufMutexOdom);
    odomBuf.push(odom_msg);
  }

  cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr;
      // 自动处理编码，如果是深度图，保持原样；如果是彩色图，转为BGR8（OpenCV默认）
      if (img_msg->encoding == "16UC1" || img_msg->encoding == "32FC1") {
          cv_ptr = cv_bridge::toCvShare(img_msg, img_msg->encoding);
      } else {
          cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8"); 
      }
      return cv_ptr->image.clone();
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return cv::Mat();
    }
  }

  void SyncWithImu() {
    const double maxTimeDiff = 0.033; 
    
    // 用于限频打印调试信息
    int debug_print_counter = 0; 

    while (rclcpp::ok()) {
      cv::Mat imRgb, imDepth;
      double tImRgb = 0, tImDepth = 0;

      if (!imgRgbBuf.empty() && !imgDepthBuf.empty() && !imuBuf.empty()) {
        tImRgb = imgRgbBuf.front()->header.stamp.sec +
                 imgRgbBuf.front()->header.stamp.nanosec * 1e-9;
        tImDepth = imgDepthBuf.front()->header.stamp.sec +
                   imgDepthBuf.front()->header.stamp.nanosec * 1e-9;

        {
          std::lock_guard<std::mutex> lock(mBufMutexDepth);
          while ((tImRgb - tImDepth) > maxTimeDiff && imgDepthBuf.size() > 1) {
            imgDepthBuf.pop();
            tImDepth = imgDepthBuf.front()->header.stamp.sec +
                       imgDepthBuf.front()->header.stamp.nanosec * 1e-9;
          }
        }

        {
          std::lock_guard<std::mutex> lock(mBufMutexRgb);
          while ((tImDepth - tImRgb) > maxTimeDiff && imgRgbBuf.size() > 1) {
            imgRgbBuf.pop();
            tImRgb = imgRgbBuf.front()->header.stamp.sec +
                     imgRgbBuf.front()->header.stamp.nanosec * 1e-9;
          }
        }

        if (std::abs(tImRgb - tImDepth) > maxTimeDiff) {
            // 时间戳对不上，跳过
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        
        double tLastImu = imuBuf.back()->header.stamp.sec +
                          imuBuf.back()->header.stamp.nanosec * 1e-9;
                          
        if (tImRgb > tLastImu) {
            // 计算滞后时间
            double lag = tImRgb - tLastImu;
            
            // 如果滞后超过0.05秒，打印警告
            if (lag > 0.05) {
                printf("\r[诊断] 图像比IMU快: %.4f 秒 (IMU数据可能中断)   ", lag);
                fflush(stdout);
            }
            
            // 重要：加一点点休眠，防止死循环占满CPU导致更严重的卡顿
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        // 取出图像
        {
          std::lock_guard<std::mutex> lock(mBufMutexRgb);
          imRgb = GetImage(imgRgbBuf.front());
          imgRgbBuf.pop();
        }

        {
          std::lock_guard<std::mutex> lock(mBufMutexDepth);
          imDepth = GetImage(imgDepthBuf.front());
          imgDepthBuf.pop();
        }

        // 取出 IMU 数据
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
          std::lock_guard<std::mutex> lock(mBufMutex);
          while (!imuBuf.empty() &&
                 imuBuf.front()->header.stamp.sec +
                         imuBuf.front()->header.stamp.nanosec * 1e-9 <=
                     tImRgb + shift) {
            double t = imuBuf.front()->header.stamp.sec +
                       imuBuf.front()->header.stamp.nanosec * 1e-9;
            cv::Point3f acc(imuBuf.front()->linear_acceleration.x,
                            imuBuf.front()->linear_acceleration.y,
                            imuBuf.front()->linear_acceleration.z);
            cv::Point3f gyr(imuBuf.front()->angular_velocity.x,
                            imuBuf.front()->angular_velocity.y,
                            imuBuf.front()->angular_velocity.z);
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
            imuBuf.pop();
          }
        }

        // 取出 Odom 数据
        vector<Eigen::Vector3f> vOdomMeas;
        {
          std::lock_guard<std::mutex> lock(mBufMutexOdom);
          while (!odomBuf.empty() &&
                 odomBuf.front()->header.stamp.sec +
                         odomBuf.front()->header.stamp.nanosec * 1e-9 <=
                     tImRgb + shift) {
            // 注意：这里我们只取线速度 x,y,z。GeoFlowSlam 需要的是速度测量。
            Eigen::Vector3f vel(odomBuf.front()->twist.twist.linear.x,
                                odomBuf.front()->twist.twist.linear.y,
                                odomBuf.front()->twist.twist.linear.z);
            vOdomMeas.push_back(vel);
            odomBuf.pop();
          }
        }

        // 调试：打印图像信息 (每30帧打印一次，避免刷屏)
        debug_print_counter++;
        if (debug_print_counter % 30 == 0) {
            double minVal, maxVal;
            cv::minMaxLoc(imDepth, &minVal, &maxVal);
            int valid_depth_pixels = cv::countNonZero(imDepth > 0);
            
            std::cout << "\n[DEBUG Info]" << std::endl;
            std::cout << "  RGB Size: " << imRgb.cols << "x" << imRgb.rows << std::endl;
            std::cout << "  Depth Size: " << imDepth.cols << "x" << imDepth.rows << std::endl;
            std::cout << "  Depth Range (mm/m): [" << minVal << ", " << maxVal << "]" << std::endl;
            std::cout << "  Valid Depth Pixels: " << valid_depth_pixels << " (如果接近0则初始化必败)" << std::endl;
            
            // 警告：如果输入尺寸和 640x480 不一致，原来的代码强制 resize 会导致畸变严重
            if (imRgb.cols != 640 || imRgb.rows != 480) {
                std::cout << "  [WARNING] 输入图像尺寸不是 640x480！YAML配置可能不匹配！" << std::endl;
            }
        }
        
        cv::Mat rgb_input = imRgb;
        cv::Mat depth_input = imDepth;

        // 如果你想强制用 640x480，请确保你的 YAML 也是 640x480 并且相机长宽比是 4:3
        // 如果你的相机是 16:9 (如 848x480)，强制 resize 到 640x480 是错误的。
        // 下面这段代码保留了原意，但如果尺寸本来就对，就不乱动。
        if (imRgb.cols != 640 || imRgb.rows != 480) {
             cv::Size dsize = cv::Size(640, 480);
             cv::resize(imRgb, rgb_input, dsize, 0, 0, cv::INTER_LINEAR); // RGB用线性插值更好
             cv::resize(imDepth, depth_input, dsize, 0, 0, cv::INTER_NEAREST); // 深度必须用最近邻
        }
        
        // ===================== 【核心修复】深度图“一刀切”物理截断 =====================
        // 在传给 SLAM 之前，直接把 5 米开外的深度数据抹零 (0 代表无效像素)
        // 这样 SLAM 和“伪雷达”就绝对不可能在 5 米外提取出包含极大坐标的噪点
        if (depth_input.type() == CV_16U) {
            // RealSense D435i 默认的 16UC1 格式，单位是毫米 (5米 = 5000毫米)
            // THRESH_TOZERO_INV 的意思是：大于 5000 的全部变成 0，小于等于 5000 的保持原样
            cv::threshold(depth_input, depth_input, 5000.0, 0, cv::THRESH_TOZERO_INV);
        } 
        else if (depth_input.type() == CV_32F) {
            // 如果深度图是 32FC1 格式，单位通常是米 (5.0米)
            cv::threshold(depth_input, depth_input, 5.0, 0, cv::THRESH_TOZERO_INV);
        }
        // ==============================================================================

        // 调用 SLAM
        {
          // 这里传入处理后的图像
          mpSLAM->TrackRGBD(rgb_input, depth_input, tImRgb, vImuMeas, vOdomMeas);
        }

        // 简单的延时策略
        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
      } else {
          // 缓冲区为空时，稍微等待一下，避免空转 CPU 占用过高
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }
// 处理并发布稀疏特征点云
  void PublishSparseMap() {
      if (!mpSLAM) return;

      // 1. 从系统中获取所有的 MapPoints
      std::vector<ORB_SLAM3::MapPoint*> vpMPs = mpSLAM->GetAllMapPoints();
      if (vpMPs.empty()) return;

      // 2. 创建 PCL 点云容器
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

      // 3. 提取有效的 3D 坐标
      for (size_t i = 0; i < vpMPs.size(); i++) {
          ORB_SLAM3::MapPoint* pMP = vpMPs[i];
          
          // 必须剔除空指针和被 BA 标记为 Bad（剔除）的点
          if (pMP && !pMP->isBad()) {
              Eigen::Vector3f pos = pMP->GetWorldPos();
              pcl::PointXYZ p;
              p.x = pos(0);
              p.y = pos(1);
              p.z = pos(2);
              cloud->points.push_back(p);
          }
      }

      if (cloud->empty()) return;

      cloud->width = cloud->points.size();
      cloud->height = 1;
      cloud->is_dense = true;

      // 4. 转换为 ROS2 的 PointCloud2 并发布
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud, cloud_msg);
      
      cloud_msg.header.stamp = this->now();
      cloud_msg.header.frame_id = "map"; // 必须与全局固定坐标系匹配

      sparse_map_pub_->publish(cloud_msg);
  }
 private:
  // 点云发布私有成员
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sparse_map_pub_;
  rclcpp::TimerBase::SharedPtr sparse_map_timer_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  std::queue<sensor_msgs::msg::Image::SharedPtr> imgRgbBuf, imgDepthBuf;
  std::mutex mBufMutexRgb, mBufMutexDepth;
  
  std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  std::mutex mBufMutex;
  
  std::queue<nav_msgs::msg::Odometry::SharedPtr> odomBuf;
  std::mutex mBufMutexOdom;
  
  ORB_SLAM3::System *mpSLAM;
  
  const bool do_rectify;
  const bool mbClahe;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc < 4) { // 需要3个参数: voc, settings, savedir
    std::cerr << "Usage: ros2 run orbslam3 rgbd_inertial_ros2 path_to_vocabulary path_to_settings path_to_save_dir" << std::endl;
    return 1;
  }

  // 初始化 SLAM 系统
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true, 0, std::string(), std::string(argv[3]));
  
  auto image_grabber = std::make_shared<ImageGrabber>(&SLAM, false, false);

  std::thread sync_thread(&ImageGrabber::SyncWithImu, image_grabber);

  rclcpp::spin(image_grabber);
  
  // 退出处理
  rclcpp::shutdown();
  sync_thread.join();
  SLAM.Shutdown(std::string(argv[3]));
  // 在程序结束时保存最终的稀疏点云
  std::cout << "正在保存最终的稀疏特征点云..." << std::endl;
  
  // 1. 获取系统中所有的 MapPoints
  std::vector<ORB_SLAM3::MapPoint*> vpMPs = SLAM.GetAllMapPoints();
  pcl::PointCloud<pcl::PointXYZ> final_cloud;
  
  // 2. 遍历并提取有效的三维坐标
  for (size_t i = 0; i < vpMPs.size(); i++) {
      if (vpMPs[i] && !vpMPs[i]->isBad()) {
          Eigen::Vector3f pos = vpMPs[i]->GetWorldPos();
          final_cloud.points.push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
      }
  }
  
  final_cloud.width = final_cloud.points.size();
  final_cloud.height = 1;
  final_cloud.is_dense = true;

  // 3. 将点云保存到你启动节点时传入的 save_dir 路径下
  if (!final_cloud.empty()) {
      std::string save_path = std::string(argv[3]) + "/sparse_map.pcd";
      // 使用 ASCII 格式保存，方便查看；如果嫌文件大可以用 savePCDFileBinary
      pcl::io::savePCDFileASCII(save_path, final_cloud);
      std::cout << "\033[1;32m[成功] 稀疏点云已保存至: \033[0m" << save_path << std::endl;
  } else {
      std::cout << "\033[1;31m[警告] 点云为空，未保存。\033[0m" << std::endl;
  }
  
  return 0;
}