#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/statistical_outlier_removal.h>

class CloudToGridNode : public rclcpp::Node {
public:
    CloudToGridNode() : Node("pointcloud_to_gridmap_node") {
        // 参数声明 (以 0.02m 分辨率为例)
        this->declare_parameter("resolution", 0.02);
        this->declare_parameter("grid_size", 10000); 
        this->declare_parameter("min_height", -0.5); 
        this->declare_parameter("max_height", 0.8);
        
        // 连通核：用于把离散的黑色点缝合成连续的垄
        this->declare_parameter("connect_size_x", 2);  
        this->declare_parameter("connect_size_y", 20); 
        
        // 拓宽核：用于让白色路径变宽（削瘦黑色障碍物）
        this->declare_parameter("widen_size", 5);     

        resolution_ = this->get_parameter("resolution").as_double();
        grid_size_ = this->get_parameter("grid_size").as_int();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();
        connect_size_x_ = this->get_parameter("connect_size_x").as_int();
        connect_size_y_ = this->get_parameter("connect_size_y").as_int();
        widen_size_ = this->get_parameter("widen_size").as_int();

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        qos.reliable();

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/slam/sparse_map_points", qos, 
            std::bind(&CloudToGridNode::CloudCallback, this, std::placeholders::_1));

        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/slam/grid_map", qos);
    }

private:
    void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>()); 
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z"); 
        pass.setFilterLimits(min_height_, max_height_);
        pass.filter(*cloud_passthrough);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>()); 
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_passthrough); 
        sor.setMeanK(40);             
        sor.setStddevMulThresh(0.5);  
        sor.filter(*cloud_filtered);

        // ================= 【核心修改区】 =================
        // 1. 初始化为“全白”背景（255代表白色，即可通行的路径）
        cv::Mat grid_image(grid_size_, grid_size_, CV_8UC1, cv::Scalar(255));
        
        float origin_x = - (grid_size_ / 2.0) * resolution_;
        float origin_y = - (grid_size_ / 2.0) * resolution_;

        // 2. 将滤波后的点投影为“黑色”障碍物 (0代表黑色)
        for (const auto& pt : cloud_filtered->points) { 
            int px = static_cast<int>((pt.x - origin_x) / resolution_);
            int py = static_cast<int>((pt.y - origin_y) / resolution_);

            if (px >= 0 && px < grid_size_ && py >= 0 && py < grid_size_) {
                grid_image.at<uchar>(py, px) = 0; // 画上黑色障碍物
            }
        }

        // 3. 第一步：缝合/连通黑色的点
        if (connect_size_x_ > 0 || connect_size_y_ > 0) {
            cv::Mat connect_kernel = cv::getStructuringElement(cv::MORPH_RECT, 
                cv::Size(connect_size_x_, connect_size_y_));
            // 腐蚀白色背景 = 放大黑色像素 = 让黑色的散点连成一堵墙
            cv::erode(grid_image, grid_image, connect_kernel);
        }

        // 4. 第二步：拓宽白色的路径
        if (widen_size_ > 0) {
            cv::Mat widen_kernel = cv::getStructuringElement(cv::MORPH_RECT, 
                cv::Size(widen_size_, widen_size_));
            // 膨胀白色背景 = 压缩黑色像素 = 把障碍物削薄，让白色路径变宽！
            cv::dilate(grid_image, grid_image, widen_kernel);
        }
        // ==================================================

        // 5. 转换为 ROS OccupancyGrid 消息
        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = this->now();
        grid_msg.header.frame_id = "map"; 
        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = grid_size_;
        grid_msg.info.height = grid_size_;
        
        grid_msg.info.origin.position.x = origin_x;
        grid_msg.info.origin.position.y = origin_y;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;

        grid_msg.data.resize(grid_size_ * grid_size_, 0); 

        for (int i = 0; i < grid_image.rows; i++) {
            for (int j = 0; j < grid_image.cols; j++) {
                // 如果像素是黑色的 (0)，说明它是障碍物
                if (grid_image.at<uchar>(i, j) == 0) {
                    grid_msg.data[i * grid_size_ + j] = 100; // 写入 100 致命障碍
                } else {
                    grid_msg.data[i * grid_size_ + j] = 0;   // 写入 0 自由路径
                }
            }
        }

        grid_pub_->publish(grid_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    double resolution_;
    int grid_size_;
    double min_height_;
    double max_height_;
    int connect_size_x_;
    int connect_size_y_;
    int widen_size_; 
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudToGridNode>());
    rclcpp::shutdown();
    return 0;
}