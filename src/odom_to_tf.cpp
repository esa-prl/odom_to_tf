#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/qos.hpp"

using std::placeholders::_1;

class OdomToTfNode : public rclcpp::Node
{
  public:
    OdomToTfNode()
    : Node("odom_to_tf")
    {
      this->declare_parameter<std::string>("gazebo_entity", "marta");

      std::string gazebo_entity;

      this->get_parameter("gazebo_entity", gazebo_entity);

      // The subscription requires sensor data quality of service. Otherwise the messages are not received.
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + gazebo_entity + "/odom", rclcpp::SensorDataQoS(), std::bind(&OdomToTfNode::topic_callback, this, _1));
      publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", tf2_ros::DynamicBroadcasterQoS());
    }

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;

    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
      geometry_msgs::msg::TransformStamped transform_msg;

      transform_msg.header = msg->header;
      transform_msg.header.frame_id = "odom";
      transform_msg.child_frame_id = "base_link";

      geometry_msgs::msg::Vector3 vector;
      vector.x = msg->pose.pose.position.x;
      vector.y = msg->pose.pose.position.y;
      vector.z = msg->pose.pose.position.z;

      transform_msg.transform.translation = vector;
      transform_msg.transform.rotation = msg->pose.pose.orientation;

      tf2_msgs::msg::TFMessage tf_msg;

      tf_msg.transforms.push_back(transform_msg);

      publisher_->publish(tf_msg);
    }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTfNode>());
  rclcpp::shutdown();
  return 0;
}
