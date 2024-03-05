#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Mapper : public rclcpp::Node
{
public:
    Mapper() : Node("converter")
    {
        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/diff_cont/cmd_vel_unstamped", 10);
        subscriber = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&Mapper::sub_callback, this, std::placeholders::_1));
    }

private:
    void sub_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::Twist new_msg;
        new_msg.linear = msg->linear;
        new_msg.angular = msg->angular;
        publisher->publish(new_msg);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
};
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mapper>());
  rclcpp::shutdown();
    return 0;
}