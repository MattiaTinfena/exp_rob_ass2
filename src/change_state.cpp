#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <memory>
#include <chrono>
#include <string>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <array>
#include <iostream>

using namespace std::chrono_literals;

class ChangeStateAction : public plansys2::ActionExecutorClient
{
public:
  ChangeStateAction()
  : plansys2::ActionExecutorClient("change_to_acquire_state", 500ms)
  {

  }

private:
  void do_work() override{
    auto args = get_arguments();
    if (args.size() == 0) {
      RCLCPP_ERROR(get_logger(), "Not enough arguments for change state action");
      finish(false, 0.0, "Insufficient arguments");
      return;
    }
    std::cout << "CHANGE STATE";
    finish(true, 1.0, "State changed");
 
  }
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);

  auto change_state_node = std::make_shared<ChangeStateAction>();
  change_state_node->set_parameter(rclcpp::Parameter("action_name", "change_to_acquire_state"));
  change_state_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  change_state_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  
  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(change_state_node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}