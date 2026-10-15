#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// PCL includes
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

class LidarFilterNode : public rclcpp::Node
{
public:
    LidarFilterNode() : Node("lidar_filter_node")
    {
        // Declare and initialize ROS 2 parameters
        this->declare_parameter<std::string>("input_topic", "/velodyne_points");
        this->declare_parameter<std::string>("output_topic", "/velodyne_points_filtered");
        this->declare_parameter<double>("pt_min_z", -1.0);
        this->declare_parameter<double>("pt_max_z", 2.0);
        this->declare_parameter<int>("sor_mean_k", 50);
        this->declare_parameter<double>("sor_stddev_mul", 1.0);

        // Fetch current parameter values
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        // Create publisher and subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10, std::bind(&LidarFilterNode::cloud_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "LidarFilterNode has started. Filtering '%s' -> '%s'", 
                    input_topic.c_str(), output_topic.c_str());
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        // 1. Convert ROS 2 message to PCL point cloud (using PointXYZI for Velodyne data)
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            return;
        }

        // 2. Step 1: Apply Pass-Through Filter
        double min_z = this->get_parameter("pt_min_z").as_double();
        double max_z = this->get_parameter("pt_max_z").as_double();

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_z, max_z);
        pass.filter(*cloud_passthrough);

        if (cloud_passthrough->empty()) {
            return;
        }

        // 3. Step 2: Apply Statistical Outlier Removal (SOR) Filter
        int mean_k = this->get_parameter("sor_mean_k").as_int();
        double stddev_mul = this->get_parameter("sor_stddev_mul").as_double();

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(cloud_passthrough);
        sor.setMeanK(mean_k);
        sor.setStddevMulThresh(stddev_mul);
        sor.filter(*cloud_filtered);

        // 4. Convert back to ROS 2 message and publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header; // Retain original frame_id and timestamp

        publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFilterNode>());
    rclcpp::shutdown();
    return 0;
}

