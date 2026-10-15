#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class VelodynePclProcessor : public rclcpp::Node {
public:
    VelodynePclProcessor() : Node("velodyne_pcl_processor") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&VelodynePclProcessor::cloud_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed_points", 10);

        RCLCPP_INFO(this->get_logger(), "Voxel Grid Downsampling node started.");
    }
private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. Convert ROS2 message to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // 2. Process the Point Cloud (Example: Voxel Grid Downsampling)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.05f, 0.05f, 0.05f);
        sor.filter(*cloud_filtered);

        // 3. Convert back to ROS2 Message and Publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header; // Keep the same frame (e.g., velodyne)
        publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelodynePclProcessor>());
    rclcpp::shutdown();
    return 0;
}
