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
        // 声明并获取参数
        this->declare_parameter("resolution", 0.05);
        this->declare_parameter("grid_size", 10000); 
        this->declare_parameter("min_height", -0.6); 
        this->declare_parameter("max_height", 1.0);
        
        // 【修改点1】：将单一的 dilate_size 拆分为水平和竖直两个参数
        this->declare_parameter("dilate_size_x", 2);  // 水平方向的核宽度（保持较小，防止左右墙粘连）
        this->declare_parameter("dilate_size_y", 13); // 竖直方向的核高度（设置较大，用于强力连通前后的断点）

        resolution_ = this->get_parameter("resolution").as_double();
        grid_size_ = this->get_parameter("grid_size").as_int();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();
        dilate_size_x_ = this->get_parameter("dilate_size_x").as_int();
        dilate_size_y_ = this->get_parameter("dilate_size_y").as_int();

        // 订阅和发布 QoS
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        qos.reliable();

        // 订阅稀疏点云话题
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/slam/sparse_map_points", qos, 
            std::bind(&CloudToGridNode::CloudCallback, this, std::placeholders::_1));

        // 发布处理后的栅格地图
        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/slam/grid_map", qos);

        RCLCPP_INFO(this->get_logger(), "点云转栅格地图节点已启动！");
    }

private:
    void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. 将 ROS2 PointCloud2 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        // 2. 高度直通滤波 (PassThrough Filter) 剔除天花板和地面
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>()); 
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z"); 
        pass.setFilterLimits(min_height_, max_height_);
        pass.filter(*cloud_passthrough);

        // 3. 统计滤波：专杀远端稀疏噪点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>()); 
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_passthrough); 
        sor.setMeanK(30);             
        sor.setStddevMulThresh(0.5);  
        sor.filter(*cloud_filtered);

        // 4. 准备 OpenCV 矩阵用于 2D 投影 (初始化为 0，代表背景)
        cv::Mat grid_image = cv::Mat::zeros(grid_size_, grid_size_, CV_8UC1);
        
        float origin_x = - (grid_size_ / 2.0) * resolution_;
        float origin_y = - (grid_size_ / 2.0) * resolution_;

        // 将滤波后的点云投影到 2D 平面
        for (const auto& pt : cloud_filtered->points) { 
            int px = static_cast<int>((pt.x - origin_x) / resolution_);
            int py = static_cast<int>((pt.y - origin_y) / resolution_);

            if (px >= 0 && px < grid_size_ && py >= 0 && py < grid_size_) {
                grid_image.at<uchar>(py, px) = 255; // 标记障碍物为白色
            }
        }

        // 5. 【核心修改】：形态学闭运算 (Closing) 配合 非对称核
        if (dilate_size_x_ > 0 || dilate_size_y_ > 0) {
            // 创建一个非对称的长方形核：宽 x，高 y
            cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, 
                cv::Size(dilate_size_x_, dilate_size_y_));
            
            // 使用形态学闭运算 (MORPH_CLOSE) 代替纯膨胀。
            // 它会强行缝合竖直方向相距不超过 dilate_size_y_ 的点，但不会显著增加墙壁的最终厚度。
            cv::morphologyEx(grid_image, grid_image, cv::MORPH_CLOSE, close_kernel);
            
            // （可选增益）由于闭运算会收缩边缘，如果导致地图在RViz中太细看不清，
            // 可以叠加一个非常轻微的全向膨胀，仅仅为了视觉加粗
            cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::dilate(grid_image, grid_image, dilate_kernel);
        }

        // 6. 转换为 ROS OccupancyGrid 消息
        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = this->now();
        grid_msg.header.frame_id = "map"; 
        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = grid_size_;
        grid_msg.info.height = grid_size_;
        
        grid_msg.info.origin.position.x = origin_x;
        grid_msg.info.origin.position.y = origin_y;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.x = 0.0;
        grid_msg.info.origin.orientation.y = 0.0;
        grid_msg.info.origin.orientation.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;

        grid_msg.data.resize(grid_size_ * grid_size_, 0); 

        for (int i = 0; i < grid_image.rows; i++) {
            for (int j = 0; j < grid_image.cols; j++) {
                if (grid_image.at<uchar>(i, j) == 255) {
                    grid_msg.data[i * grid_size_ + j] = 100; // 100 代表有障碍物
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
    // 替换原有的 dilate_size_
    int dilate_size_x_;
    int dilate_size_y_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudToGridNode>());
    rclcpp::shutdown();
    return 0;
}