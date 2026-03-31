#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/opencv.hpp>

class CloudToGridNode : public rclcpp::Node {
public:
    CloudToGridNode() : Node("pointcloud_to_gridmap_node") {
        // 声明并获取参数（支持启动时动态修改）
        this->declare_parameter("resolution", 0.05);
        this->declare_parameter("grid_size", 10000); // 1000*0.05 = 50米
        this->declare_parameter("min_height", -0.6); // 针对相机坐标系调整
        this->declare_parameter("max_height", 1.0);
        this->declare_parameter("dilate_size", 2);   // 膨胀核大小，用于加粗特征点

        resolution_ = this->get_parameter("resolution").as_double();
        grid_size_ = this->get_parameter("grid_size").as_int();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();
        dilate_size_ = this->get_parameter("dilate_size").as_int();

        // 订阅和发布 QoS
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        qos.reliable();

        // 订阅你刚才写的稀疏点云话题
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        // 注意：如果你发现高度轴是 Z，请把 "y" 改成 "z"
        pass.setFilterFieldName("z"); 
        pass.setFilterLimits(min_height_, max_height_);
        pass.filter(*cloud_filtered);

        // 3. 准备 OpenCV 矩阵用于 2D 投影 (初始化为 0，代表背景)
        cv::Mat grid_image = cv::Mat::zeros(grid_size_, grid_size_, CV_8UC1);
        
        float origin_x = - (grid_size_ / 2.0) * resolution_;
        float origin_y = - (grid_size_ / 2.0) * resolution_;

        // 4. 将点云投影到 2D 平面
        for (const auto& pt : cloud_filtered->points) {
            // 假设机器人在 X-Z 平面移动 (ORB-SLAM3 默认)
            // 如果你的机器人是在 X-Y 平面移动，请改成 pt.x 和 pt.y
            int px = static_cast<int>((pt.x - origin_x) / resolution_);
            int py = static_cast<int>((pt.y - origin_y) / resolution_);

            if (px >= 0 && px < grid_size_ && py >= 0 && py < grid_size_) {
                grid_image.at<uchar>(py, px) = 255; // 标记障碍物为白色
            }
        }

        // 5. 形态学膨胀 (Dilation) - 让稀疏的点连成线或块，避免 RViz 里看不清
        if (dilate_size_ > 0) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, 
                cv::Size(dilate_size_, dilate_size_));
            cv::dilate(grid_image, grid_image, kernel);
        }

        // 6. 转换为 ROS OccupancyGrid 消息
        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = this->now();
        grid_msg.header.frame_id = "map"; // 必须与点云 frame_id 一致
        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = grid_size_;
        grid_msg.info.height = grid_size_;
        grid_msg.info.origin.orientation.w = 1.0;
        
        // 大概在第 85 行左右，把之前我让你改的四元数删掉，换回全 0 和 w=1.0
        grid_msg.info.origin.position.x = origin_x;
        grid_msg.info.origin.position.y = origin_y;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.x = 0.0;
        grid_msg.info.origin.orientation.y = 0.0;
        grid_msg.info.origin.orientation.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;

        grid_msg.data.resize(grid_size_ * grid_size_, -1); // 默认 -1 (未知区域)

        for (int i = 0; i < grid_image.rows; i++) {
            for (int j = 0; j < grid_image.cols; j++) {
                if (grid_image.at<uchar>(i, j) == 255) {
                    grid_msg.data[i * grid_size_ + j] = 100; // 100 代表有障碍物
                } else {
                    // 稀疏建图一般无法明确知道哪里是 Free (0)
                    // 所以这里保持背景为 Unknown (-1)，如果你想强行把没点的地方当做可通行，可以设为 0
                    grid_msg.data[i * grid_size_ + j] = -1; 
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
    int dilate_size_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudToGridNode>());
    rclcpp::shutdown();
    return 0;
}