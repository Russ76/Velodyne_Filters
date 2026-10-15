#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class SorFilterNode : public rclcpp::Node
{
public:
  SorFilterNode() : Node("sor_filter_node")
  {
    // Declare parameters for dynamic tuning
    this->declare_parameter<int>("meank", 50);
    this->declare_parameter<double>("stddev_mul", 1.0);

    // Velodyne VLP-16 typically publishes on /velodyne_points
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10, std::bind(&SorFilterNode::cloud_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);
    
    RCLCPP_INFO(this->get_logger(), "Statistical Outlier Removal Node Started.");
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 1. Convert ROS 2 message to PCL PointCloud (using PointXYZI for intensity data from VLP-16)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      return;
    }

    // 2. Initialize the Statistical Outlier Removal Filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);

    // Get current filter parameters
    int meank = this->get_parameter("meank").as_int();
    double stddev_mul = this->get_parameter("stddev_mul").as_double();

    // 3. Set Filter Configuration
    sor.setMeanK(meank);            // Number of neighbors to analyze per point
    sor.setStddevMulThresh(stddev_mul); // Standard deviation multiplier threshold
    
    // 4. Execute Filtering
    sor.filter(*cloud_filtered);

    // 5. Convert filtered PCL data back to ROS 2 message
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_filtered, output_msg);
    output_msg.header = msg->header; // Preserve timestamp and frame_id (e.g., velodyne)

    // Publish the clean point cloud
    publisher_->publish(output_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SorFilterNode>());
  rclcpp::shutdown();
  return 0;
}
