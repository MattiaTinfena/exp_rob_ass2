#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include <memory>
#include <chrono>
#include <string>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <array>

struct Point {
  double x;
  double y;

  // Point()
  
  // Point(double x, double y) {
  //   this->x = x;
  //   this->y = y;
  // }
};

#define INT_POINTS 3

using namespace std::chrono_literals;

class DetectIdAction : public plansys2::ActionExecutorClient
{
public:
  DetectIdAction()
  : plansys2::ActionExecutorClient("detect_id", 500ms), goal_sent_(false), progress_(0.0)
  {
    odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&DetectIdAction::odom_callback, this, std::placeholders::_1)
    );

    nav2_node_ = rclcpp::Node::make_shared("move_action_nav2_client");
    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      nav2_node_, "navigate_to_pose"
    );
  }

private:
  void do_work() override
  {
    auto args = get_arguments();
    if (args.size() == 0) {
      RCLCPP_ERROR(get_logger(), "Not enough arguments for move action");
      finish(false, 0.0, "Insufficient arguments");
      return;
    }

    std::unordered_map<std::string, Point> goals=
    {
        {"p1", {-6.0, -4.5}},
        {"p2", {-6.0, 7.5}},
        {"p3", {6.0, -4.5}},
        {"p4", {6.0, 7.5}},
        {"base", {0.0, 0.0}}
    };

    std::string wp_to_navigate = args[1];
    std::string wp_starting_point = args[2];


    // double goal_x, goal_y;
    // if (wp_to_navigate == "p1") {
    //   goal_x = -6.0;
    //   goal_y = -4.5;
    // } else if (wp_to_navigate == "p2") {
    //   goal_x = -6.0;
    //   goal_y = 7.5;
    // }else if (wp_to_navigate == "p3") {
    //   goal_x = 6.0;
    //   goal_y = -4.5;
    // }else if (wp_to_navigate == "p4") {
    //   goal_x = 6.0;
    //   goal_y = 7.5;
    // }else if (wp_to_navigate == "base") {
    //   goal_x = 0.0;
    //   goal_y = 0.0;
    // } else {
    //   RCLCPP_ERROR(get_logger(), "Unknown waypoint: %s", wp_to_navigate.c_str());
    //   finish(false, 0.0, "Unknown waypoint");
    //   return;
    // }



    if (!goal_sent_) {
      if (!nav2_client_->wait_for_action_server(1s)) {
        RCLCPP_WARN(get_logger(), "NavigateToPose server not ready");
        return;
      }

      const Point starting = goals[wp_starting_point];
      const Point ending = goals[wp_to_navigate];

      std::array<Point, INT_POINTS + 1> intermediate_points;
      std::cout << "Starting point " << "x: " << starting.x << "y: " << starting.y << std::endl;


      for (int i = 0; i < INT_POINTS; ++i) {
        intermediate_points[i].x = ((INT_POINTS - i) * starting.x + (i + 1) * ending.x) / (INT_POINTS + 1.0);
        intermediate_points[i].y = ((INT_POINTS - i) * starting.y + (i + 1) * ending.y) / (INT_POINTS + 1.0);
        std::cout << "Intermediate point " << i << "x: " << intermediate_points[i].x << "y: " << intermediate_points[i].y << std::endl;
      }
      intermediate_points[intermediate_points.size() - 1] = ending;
      std::cout << "End point " << "x: " << ending.x << "y: " << ending.y << std::endl;

      int goal_idx = 0;

      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.frame_id = "map";
      goal_pose.pose.position.x = intermediate_points[0].x;
      goal_pose.pose.position.y = intermediate_points[0].y;
      goal_pose.pose.orientation.w = 1.0;

      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose = goal_pose;

      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
      auto callback = [this, intermediate_points, &goal_idx, &send_goal_options]
        (const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
        {
          ++goal_idx;
          std::cout << "Setting starting point with idx: " << goal_idx << std::endl;

          if (goal_idx >= intermediate_points.size()) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
              this->goal_sent_= false;
              progress_ = 1.0;
              // RCLCPP_INFO(get_logger(), "Reached waypoint:");
              finish(true, 1.0, "Move completed");
            } else {
              // RCLCPP_ERROR(get_logger(), "Navigation failed");
              finish(true, 1.0, "Move failed");
            }
            return;
          }
          
          geometry_msgs::msg::PoseStamped goal_pose;
          goal_pose.header.frame_id = "map";
          goal_pose.pose.position.x = intermediate_points[goal_idx].x;
          goal_pose.pose.position.y = intermediate_points[goal_idx].y;
          goal_pose.pose.orientation.w = 1.0;
          auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
          goal_msg.pose = goal_pose;

          this->nav2_client_->async_send_goal(goal_msg, send_goal_options);
        };

      send_goal_options.result_callback = callback;
        
      nav2_client_->async_send_goal(goal_msg, send_goal_options);
      goal_sent_ = true;

      start_x_ = current_x_;
      start_y_ = current_y_;
    }

    // double total_dist = std::hypot(goal_x - start_x_, goal_y - start_y_);
    // double rem_dist   = std::hypot(goal_x - current_x_, goal_y - current_y_);
    // progress_ = total_dist > 0.0 ? 1.0 - std::min(rem_dist / total_dist, 1.0) : 1.0;

    send_feedback(progress_, "Moving to " + wp_to_navigate);

    // if (rem_dist < 0.6) {
    //   goal_sent_= false;
    //   progress_ = 1.0;
    //   send_feedback(progress_, "Moving to " + wp_to_navigate);
    //   RCLCPP_INFO(get_logger(), "Reached waypoint: %s", wp_to_navigate.c_str());
    // }

    rclcpp::spin_some(nav2_node_);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
  }
  
  bool goal_sent_;
  float progress_;
  double start_x_ = 0.0, start_y_ = 0.0;
  double current_x_ = 0.0, current_y_ = 0.0;

  rclcpp::Node::SharedPtr nav2_node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectIdAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "detect_id"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}