#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"  // For quaternion to Euler conversion
// #include <chrono>

using namespace std::chrono_literals;

class ArticubotFSM : public rclcpp::Node
{
public:
  ArticubotFSM() : Node("fsm_main"), state_(GO_TO_A)
  {
    // Define target positions
    target_positions_[GO_TO_A] = std::make_pair(0.0, 0.0);
    target_positions_[GO_TO_B] = std::make_pair(2.0, 0.0);
    target_positions_[GO_TO_C] = std::make_pair(2.0, 2.0);
    target_positions_[BACK_TO_A] = std::make_pair(0.0, 0.0);
    
    // Tolerance for reaching waypoint
    waypoint_tolerance_ = 0.1;

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/diff_cont/cmd_vel_unstamped", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ArticubotFSM::scanCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/diff_cont/odom", 10,
        std::bind(&ArticubotFSM::odomCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(100ms, std::bind(&ArticubotFSM::update, this));
    // Timer untuk mengukur durasi berhenti
    stop_timer_ = create_wall_timer(100ms, std::bind(&ArticubotFSM::checkStopDuration, this));

    RCLCPP_INFO(get_logger(), "FSM Delivery Robot STARTED");
    RCLCPP_INFO(get_logger(), "Memulai perjalanan dari titik A...");
  }

private:
  enum State
  {
    GO_TO_A,
    STOP_AT_A,
    GO_TO_B,
    STOP_AT_B,
    GO_TO_C,
    STOP_AT_C,
    BACK_TO_A,
    STOP_AT_A_FINAL,
    AVOID_OBSTACLE
  };

  State state_;
  bool obstacle_ = false;
  bool is_stopped_ = false;
  double x_, y_, yaw_;
  double waypoint_tolerance_;
  
  // Variabel untuk mengatur durasi berhenti
  rclcpp::Time stop_start_time_;
  double stop_duration_ = 2.0; // Berhenti selama 2 detik di setiap titik
  // Store target positions for each state
  std::map<State, std::pair<double, double>> target_positions_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr stop_timer_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    
    // Get yaw from quaternion using tf2
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double yaw, roll, pitch;
    conv.quat_to_eular(q, yaw, pitch, roll);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Check a range of angles in front of robot (not just the middle)
    int center_idx = msg->ranges.size() / 2;
    int check_range = 20; // Check ±20 readings
    obstacle_ = false;
    
    for (int i = -check_range; i <= check_range; ++i) {
      int idx = center_idx + i;
      if (idx >= 0 && idx < static_cast<int>(msg->ranges.size())) {
        if (msg->ranges[idx] < 0.6 && msg->ranges[idx] > 0.1) {
          obstacle_ = true;
          break;
        }
      }
    }
  }

  double distanceToTarget(State state) {
    auto target = target_positions_[state];
    return sqrt(pow(target.first - x_, 2) + pow(target.second - y_, 2));
  }

  double angleToTarget(State state) {
    auto target = target_positions_[state];
    return atan2(target.second - y_, target.first - x_);
  }
  void startStopTimer() {
    stop_start_time_ = now();
    is_stopped_ = true;
    RCLCPP_INFO(get_logger(), "Robot BERHENTI. Menunggu %0.1f detik...", stop_duration_);
  }

  void checkStopDuration() {
    if (!is_stopped_) return;
    
    auto elapsed = (now() - stop_start_time_).seconds();
    if (elapsed >= stop_duration_) {
      is_stopped_ = false;
      
      // Transisi state setelah berhenti
      switch (state_) {
        case STOP_AT_A:
          state_ = GO_TO_B;
          RCLCPP_INFO(get_logger(), "Barang diambil di A. Menuju titik B...");
          break;
        case STOP_AT_B:
          state_ = GO_TO_C;
          RCLCPP_INFO(get_logger(), "Barang ditaruh di B. Menuju titik C...");
          break;
        case STOP_AT_C:
          state_ = BACK_TO_A;
          RCLCPP_INFO(get_logger(), "Barang ditaruh di C. Kembali ke titik A untuk ambil barang...");
          break;
        case STOP_AT_A_FINAL:
          state_ = GO_TO_A; // Kembali ke awal untuk siklus berikutnya
          RCLCPP_INFO(get_logger(), "Barang diambil di A. Siklus pengiriman selesai!");
          RCLCPP_INFO(get_logger(), "Memulai siklus baru dari titik A...");
          break;
        default:
          break;
      }
    }
  }

  void update()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    if (obstacle_ && !is_stopped_) {
      state_ = AVOID_OBSTACLE;
    }

    switch (state_)
    {
    case GO_TO_A:
    case GO_TO_B:
    case GO_TO_C:
    case BACK_TO_A:
      {
        double distance = distanceToTarget(GO_TO_A);
        double angle_error = angleToTarget(GO_TO_A) - yaw_;
        
        // Normalize angle error to [-π, π]
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
        
        if (distance < waypoint_tolerance_) {
          // Waypoint reached, go to next state
          switch (state_) {
            case GO_TO_A: state_ = GO_TO_B; RCLCPP_INFO(get_logger(), "Reached A, going to B"); break;
            case GO_TO_B: state_ = GO_TO_C; RCLCPP_INFO(get_logger(), "Reached B, going to C"); break;
            case GO_TO_C: state_ = BACK_TO_A; RCLCPP_INFO(get_logger(), "Reached C, going back to A"); break;
            case BACK_TO_A: state_ = GO_TO_A; RCLCPP_INFO(get_logger(), "Back at A, mission complete!"); break;
            default: break;
          }
        } else {
          // Navigate to waypoint
          if (fabs(angle_error) > 0.1) {
            // Turn to face target
            cmd.angular.z = 0.3 * angle_error;
          } else {
            // Move forward
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.1 * angle_error; // Small correction
          }
        }
      }
      break;

    case AVOID_OBSTACLE:
      // More sophisticated obstacle avoidance
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.5;
      
      // Only exit avoidance if obstacle is cleared AND we're facing away from it
      if (!obstacle_) {
        // Return to previous state based on where we are
        double dist_to_A = distanceToTarget(GO_TO_A);
        double dist_to_B = distanceToTarget(GO_TO_B);
        double dist_to_C = distanceToTarget(GO_TO_C);
        
        // Go to nearest waypoint
        if (dist_to_A <= dist_to_B && dist_to_A <= dist_to_C) {
          state_ = GO_TO_A;
        } else if (dist_to_B <= dist_to_A && dist_to_B <= dist_to_C) {
          state_ = GO_TO_B;
        } else {
          state_ = GO_TO_C;
        }
      }
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