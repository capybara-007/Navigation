#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cctype>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class HeightFilter : public rclcpp::Node
{
public:
    HeightFilter()
        : Node("lio_sam_height_filter"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        clock_ = this->get_clock();
        input_topic_ = declare_parameter<std::string>("input_topic", "/lio_sam/mapping/cloud_registered");
        output_topic_ = declare_parameter<std::string>("output_topic", "/lio_sam/mapping/cloud_registered_height_filtered");
        target_frame_ = declare_parameter<std::string>("target_frame", "map");
        axis_name_ = declare_parameter<std::string>("axis", "z");
        min_height_ = declare_parameter<double>("min_height", -1.0);
        max_height_ = declare_parameter<double>("max_height", 2.0);
        drop_empty_clouds_ = declare_parameter<bool>("drop_empty_clouds", false);
        transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);

        if (min_height_ > max_height_) {
            std::swap(min_height_, max_height_);
            RCLCPP_WARN(get_logger(), "min_height was greater than max_height; values were swapped.");
        }

        axis_name_ = normalizeAxisName(axis_name_);
        if (axis_name_.empty()) {
            axis_name_ = "z";
            RCLCPP_WARN(get_logger(), "Invalid axis parameter; using z.");
        }

        pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS(5));
        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            rclcpp::QoS(5),
            std::bind(&HeightFilter::cloudHandler, this, std::placeholders::_1));
        param_callback_handle_ = add_on_set_parameters_callback(
            std::bind(&HeightFilter::parametersHandler, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Filtering %s -> %s in frame '%s' on %s axis, keeping [%.3f, %.3f] m.",
            input_topic_.c_str(),
            output_topic_.c_str(),
            target_frame_.empty() ? "<input>" : target_frame_.c_str(),
            axis_name_.c_str(),
            min_height_,
            max_height_);
    }

private:
    std::string normalizeAxisName(const std::string& axis_name) const
    {
        if (axis_name.empty()) {
            return "";
        }

        char axis = static_cast<char>(std::tolower(static_cast<unsigned char>(axis_name[0])));
        if (axis == 'x' || axis == 'y' || axis == 'z') {
            return std::string(1, axis);
        }
        return "";
    }

    bool parameterAsDouble(
        const rclcpp::Parameter& parameter,
        double& value,
        std::string& reason) const
    {
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            value = parameter.as_double();
            return true;
        }
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            value = static_cast<double>(parameter.as_int());
            return true;
        }

        reason = parameter.get_name() + " must be numeric.";
        return false;
    }

    rcl_interfaces::msg::SetParametersResult parametersHandler(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        std::string target_frame = target_frame_;
        std::string axis_name = axis_name_;
        double min_height = min_height_;
        double max_height = max_height_;
        bool drop_empty_clouds = drop_empty_clouds_;
        double transform_timeout_sec = transform_timeout_sec_;

        for (const auto& parameter : parameters) {
            const std::string& name = parameter.get_name();

            if (name == "input_topic" || name == "output_topic") {
                result.successful = false;
                result.reason = name + " cannot be changed after startup.";
                return result;
            } else if (name == "target_frame") {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                    result.successful = false;
                    result.reason = "target_frame must be a string.";
                    return result;
                }
                target_frame = parameter.as_string();
            } else if (name == "axis") {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                    result.successful = false;
                    result.reason = "axis must be a string.";
                    return result;
                }
                axis_name = normalizeAxisName(parameter.as_string());
            } else if (name == "min_height") {
                if (!parameterAsDouble(parameter, min_height, result.reason)) {
                    result.successful = false;
                    return result;
                }
            } else if (name == "max_height") {
                if (!parameterAsDouble(parameter, max_height, result.reason)) {
                    result.successful = false;
                    return result;
                }
            } else if (name == "drop_empty_clouds") {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                    result.successful = false;
                    result.reason = "drop_empty_clouds must be a bool.";
                    return result;
                }
                drop_empty_clouds = parameter.as_bool();
            } else if (name == "transform_timeout_sec") {
                if (!parameterAsDouble(parameter, transform_timeout_sec, result.reason)) {
                    result.successful = false;
                    return result;
                }
            }
        }

        if (axis_name.empty()) {
            result.successful = false;
            result.reason = "axis must be x, y, or z.";
            return result;
        }
        if (min_height > max_height) {
            result.successful = false;
            result.reason = "min_height must be <= max_height.";
            return result;
        }
        if (transform_timeout_sec < 0.0) {
            result.successful = false;
            result.reason = "transform_timeout_sec must be >= 0.";
            return result;
        }

        target_frame_ = target_frame;
        axis_name_ = axis_name;
        min_height_ = min_height;
        max_height_ = max_height;
        drop_empty_clouds_ = drop_empty_clouds;
        transform_timeout_sec_ = transform_timeout_sec;

        return result;
    }

    bool findAxisOffset(const sensor_msgs::msg::PointCloud2& cloud, uint32_t& offset) const
    {
        for (const auto& field : cloud.fields) {
            if (field.name == axis_name_) {
                if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *clock_, 5000,
                        "PointCloud2 field '%s' is not FLOAT32; height filter skipped.",
                        axis_name_.c_str());
                    return false;
                }
                offset = field.offset;
                return true;
            }
        }

        RCLCPP_WARN_THROTTLE(
            get_logger(), *clock_, 5000,
            "PointCloud2 field '%s' was not found; height filter skipped.",
            axis_name_.c_str());
        return false;
    }

    bool transformCloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_in,
        sensor_msgs::msg::PointCloud2& cloud_out)
    {
        if (target_frame_.empty() || cloud_in->header.frame_id == target_frame_) {
            cloud_out = *cloud_in;
            return true;
        }

        try {
            const auto transform = tf_buffer_.lookupTransform(
                target_frame_,
                cloud_in->header.frame_id,
                rclcpp::Time(cloud_in->header.stamp),
                rclcpp::Duration::from_seconds(transform_timeout_sec_));
            tf2::doTransform(*cloud_in, cloud_out, transform);
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *clock_, 5000,
                "Could not transform cloud from '%s' to '%s': %s",
                cloud_in->header.frame_id.c_str(),
                target_frame_.c_str(),
                ex.what());
            return false;
        }
    }

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        if (!transformCloud(msg, cloud)) {
            return;
        }

        uint32_t axis_offset = 0;
        if (!findAxisOffset(cloud, axis_offset)) {
            return;
        }

        if (cloud.point_step == 0) {
            return;
        }

        if (axis_offset + sizeof(float) > cloud.point_step) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *clock_, 5000,
                "Malformed PointCloud2 field offset; height filter skipped.");
            return;
        }

        const size_t point_count = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
        if (point_count > cloud.data.size() / cloud.point_step) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *clock_, 5000,
                "Malformed PointCloud2 data; height filter skipped.");
            return;
        }

        sensor_msgs::msg::PointCloud2 filtered = cloud;
        filtered.data.clear();
        filtered.data.reserve(cloud.data.size());
        filtered.height = 1;
        filtered.width = 0;

        for (size_t i = 0; i < point_count; ++i) {
            const uint8_t* point_data = &cloud.data[i * cloud.point_step];

            float height_value = std::numeric_limits<float>::quiet_NaN();
            std::memcpy(&height_value, point_data + axis_offset, sizeof(float));

            if (!std::isfinite(height_value)) {
                continue;
            }

            const double height = static_cast<double>(height_value);
            if (height < min_height_ || height > max_height_) {
                continue;
            }

            filtered.data.insert(filtered.data.end(), point_data, point_data + cloud.point_step);
            ++filtered.width;
        }

        filtered.row_step = filtered.point_step * filtered.width;
        filtered.is_dense = cloud.is_dense;

        if (drop_empty_clouds_ && filtered.width == 0) {
            return;
        }

        pub_cloud_->publish(filtered);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
    std::string axis_name_;
    double min_height_;
    double max_height_;
    bool drop_empty_clouds_;
    double transform_timeout_sec_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::Clock::SharedPtr clock_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeightFilter>());
    rclcpp::shutdown();
    return 0;
}
