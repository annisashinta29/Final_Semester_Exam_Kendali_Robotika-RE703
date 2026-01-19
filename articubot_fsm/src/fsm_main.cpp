#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class ArticubotFSM : public rclcpp::Node
{
public:
  ArticubotFSM() : Node("fsm_main"), state_(GO_TO_A)
  {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/diff_cont/cmd_vel_unstamped", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&ArticubotFSM::scanCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(100ms, std::bind(&ArticubotFSM::update, this));
    RCLCPP_INFO(get_logger(), "FSM Delivery Robot STARTED");
  }

private:
  enum State
  {
    GO_TO_A,
    GO_TO_B,
    GO_TO_C,
    BACK_TO_A,
    AVOID_OBSTACLE
  };

  State state_;
  bool obstacle_ = false;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    obstacle_ = msg->ranges[msg->ranges.size() / 2] < 0.8; //distance
  }

  void update()
  {
    geometry_msgs::msg::Twist cmd;

    if (obstacle_)
    {
      state_ = AVOID_OBSTACLE;
    }

    switch (state_)
    {
    case GO_TO_A:
      cmd.linear.x = 0.5;
      state_ = GO_TO_B;
      break;

    case GO_TO_B:
      cmd.linear.x = 0.5;
      state_ = GO_TO_C;
      break;

    case GO_TO_C:
      cmd.linear.x = 0.5;
      state_ = BACK_TO_A;
      break;

    case BACK_TO_A:
      cmd.linear.x = 0.5;
      state_ = GO_TO_A;
      break;

    case AVOID_OBSTACLE:
      cmd.angular.z = 0.5;
      if (!obstacle_)
        state_ = GO_TO_A;
      break;
    }

    cmd_pub_->publish(cmd);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArticubotFSM>());
  rclcpp::shutdown();
  return 0;
}
