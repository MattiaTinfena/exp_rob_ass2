#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "plansys_interface/action/go_to_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>

using namespace std::chrono_literals;
class GoToPointNode : public rclcpp::Node {
  public:
	using GoToPoint = plansys_interface::action::GoToPoint;
	using GoalHandleGoToPoint = rclcpp_action::ServerGoalHandle<GoToPoint>;

	GoToPointNode() : rclcpp::Node("GoToPointNode") {

		// nav2_node_ = rclcpp::Node::make_shared("move_action_nav2_client");

		nav2_client_ =
			rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
				this, "navigate_to_pose");

		if (!nav2_client_->wait_for_action_server(15s)) {
			RCLCPP_WARN(get_logger(), "NavigateToPose server not ready");
			return;
		}
		auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid,
								  std::shared_ptr<const GoToPoint::Goal> goal) {
			RCLCPP_INFO(this->get_logger(),
						"Received goal request: x = %.2f, y = %.2f, orz = "
						"%.2f, orw = %.2f",
						goal->goal.position.x, goal->goal.position.y,
						goal->goal.orientation.z, goal->goal.orientation.w);
			(void)uuid;
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		};

		auto handle_cancel =
			[this](const std::shared_ptr<GoalHandleGoToPoint> goal_handle) {
				RCLCPP_INFO(this->get_logger(),
							"Received request to cancel goal");
				(void)goal_handle;
				return rclcpp_action::CancelResponse::ACCEPT;
			};

		auto nav2_result_callback =
			[this](const rclcpp_action::ClientGoalHandle<
				   nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
				auto result_msg = std::make_shared<GoToPoint::Result>();

				if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
					RCLCPP_ERROR(get_logger(), "Navigation failed:");
					result_msg->success = false;
					this->goal_handle->succeed(result_msg);
				} else {
					RCLCPP_INFO(get_logger(), "Navigation succeded:");
					result_msg->success = true;
					this->goal_handle->succeed(result_msg);
				}
			};
		rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::
			SendGoalOptions send_goal_options;
		send_goal_options.result_callback = nav2_result_callback;

		auto handle_accepted =
			[this, send_goal_options](
				const std::shared_ptr<GoalHandleGoToPoint> goal_handle) {
				this->goal_handle = goal_handle;
				auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
				goal_msg.pose.header.frame_id = "map";
				goal_msg.pose.pose = goal_handle->get_goal()->goal;
				nav2_client_->async_send_goal(goal_msg, send_goal_options);
			};

		this->action_server_ = rclcpp_action::create_server<GoToPoint>(
			this, "go_to_point", handle_goal, handle_cancel, handle_accepted);

		// rclcpp::spin_some(nav2_node_);
	}

  private:
	// void execute(const std::shared_ptr<GoalHandleGoToPoint> goal_handle) {
	// 	RCLCPP_INFO(this->get_logger(), "Executing GoToPoint action");

	// 	auto result = std::make_shared<GoToPoint::Result>();
	// 	result->success = true;

	// 	goal_handle->succeed(result);
	// }

	rclcpp_action::Server<GoToPoint>::SharedPtr action_server_;
	// rclcpp::Node::SharedPtr nav2_node_;
	rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
		nav2_client_;
	std::shared_ptr<GoalHandleGoToPoint> goal_handle;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<GoToPointNode>();

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
				"Ready to reach the point and align.");

	rclcpp::spin(node);
	rclcpp::shutdown();
}