#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>  // 追加

using namespace std::chrono_literals;  // 追加

class PCDPublisher : public rclcpp::Node
{
public:
  PCDPublisher() : Node("pcd_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&PCDPublisher::publishPCD, this));
  }

private:
  void publishPCD()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/porizou/bunny.pcd", cloud) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read the PCD file");
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "map";

    publisher_->publish(cloud_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCDPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
