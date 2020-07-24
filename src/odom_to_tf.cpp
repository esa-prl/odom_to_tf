#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using std::placeholders::_1;

class OdomToTfNode : public rclcpp::Node
{
  public:
    OdomToTfNode()
    : Node("odom_to_tf")
    {
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&OdomToTfNode::topic_callback, this, _1));
      publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
    }

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;

    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
      geometry_msgs::msg::TransformStamped transform_msg;

      transform_msg.header = msg->header;

      transform_msg.child_frame_id = "odom"; //parametrize

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
