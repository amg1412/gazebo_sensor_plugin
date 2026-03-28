/**
 * robot_lifecycle_node.cpp
 *
 * A ROS 2 lifecycle node that manages a single robot's state machine:
 *   Unconfigured → Inactive → Active → (navigation running) → Inactive
 *
 * Each robot instance is launched with a unique namespace (e.g. /robot1)
 * so all topics, services, and actions are fully isolated.
 *
 * State machine:
 *   on_configure  – validate namespace, set up action client to Nav2
 *   on_activate   – mark robot as ready to receive waypoint goals
 *   on_deactivate – cancel any running navigation goal
 *   on_cleanup    – release all resources
 *   on_shutdown   – emergency stop
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using NavigateToPose   = nav2_msgs::action::NavigateToPose;
using GoalHandleNav    = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using CallbackReturn   = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ── Robot states (beyond the lifecycle states) ────────────────────────────
enum class NavState { IDLE, NAVIGATING, SUCCEEDED, FAILED, CANCELLED };

static std::string nav_state_str(NavState s) {
  switch (s) {
    case NavState::IDLE:        return "IDLE";
    case NavState::NAVIGATING:  return "NAVIGATING";
    case NavState::SUCCEEDED:   return "SUCCEEDED";
    case NavState::FAILED:      return "FAILED";
    case NavState::CANCELLED:   return "CANCELLED";
    default:                    return "UNKNOWN";
  }
}

// ─────────────────────────────────────────────────────────────────────────────
class RobotLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit RobotLifecycleNode(const std::string & node_name,
                               const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode(node_name, options),
    nav_state_(NavState::IDLE)
  {
    // Declare parameters
    this->declare_parameter<std::string>("robot_namespace", "robot1");
    this->declare_parameter<double>("goal_timeout_sec", 60.0);
  }

  // ── Lifecycle transitions ──────────────────────────────────────────────

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    robot_ns_   = this->get_parameter("robot_namespace").as_string();
    goal_timeout_ = this->get_parameter("goal_timeout_sec").as_double();

    RCLCPP_INFO(get_logger(), "[%s] Configuring — namespace: /%s",
                get_name(), robot_ns_.c_str());

    // Action client — connects to Nav2 running under this robot's namespace
    // e.g.  /robot1/navigate_to_pose
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "/" + robot_ns_ + "/navigate_to_pose");

    // Status publisher so the task allocator knows what this robot is doing
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/" + robot_ns_ + "/nav_status", 10);

    RCLCPP_INFO(get_logger(), "[%s] Configured. Action server: /%s/navigate_to_pose",
                get_name(), robot_ns_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating …", get_name());

    // Wait for Nav2 action server (up to 10 s)
    if (!nav_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(),
        "[%s] Nav2 action server not available after 10 s. "
        "Is nav2_bringup running under namespace /%s?",
        get_name(), robot_ns_.c_str());
      return CallbackReturn::FAILURE;
    }

    status_pub_->on_activate();
    nav_state_ = NavState::IDLE;
    publish_status();

    RCLCPP_INFO(get_logger(), "[%s] Active — ready to receive goals.", get_name());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating …", get_name());
    cancel_current_goal();
    status_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleaning up.", get_name());
    nav_client_.reset();
    status_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_WARN(get_logger(), "[%s] Shutdown requested — cancelling goals.", get_name());
    cancel_current_goal();
    return CallbackReturn::SUCCESS;
  }

  // ── Public API used by the task allocator ─────────────────────────────

  /**
   * Send a NavigateToPose goal.
   * Returns false if the node is not active or already navigating.
   */
  bool send_goal(double x, double y, double yaw_deg = 0.0)
  {
    if (nav_state_ == NavState::NAVIGATING) {
      RCLCPP_WARN(get_logger(), "[%s] Already navigating — ignoring new goal.", get_name());
      return false;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp    = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;

    // Convert yaw (degrees) → quaternion (z, w)
    double yaw_rad = yaw_deg * M_PI / 180.0;
    goal_msg.pose.pose.orientation.z = std::sin(yaw_rad / 2.0);
    goal_msg.pose.pose.orientation.w = std::cos(yaw_rad / 2.0);

    RCLCPP_INFO(get_logger(), "[%s] Sending goal → (%.2f, %.2f, %.1f°)",
                get_name(), x, y, yaw_deg);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // Goal accepted / rejected
    send_goal_options.goal_response_callback =
      [this](const GoalHandleNav::SharedPtr & gh) {
        if (!gh) {
          RCLCPP_ERROR(get_logger(), "[%s] Goal REJECTED by Nav2.", get_name());
          nav_state_ = NavState::FAILED;
        } else {
          RCLCPP_INFO(get_logger(), "[%s] Goal ACCEPTED.", get_name());
          nav_state_ = NavState::NAVIGATING;
          goal_handle_ = gh;
        }
        publish_status();
      };

    // Feedback (distance remaining)
    send_goal_options.feedback_callback =
      [this](GoalHandleNav::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_DEBUG(get_logger(), "[%s] Distance remaining: %.2f m",
                     get_name(), fb->distance_remaining);
      };

    // Result
    send_goal_options.result_callback =
      [this](const GoalHandleNav::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "[%s] Goal SUCCEEDED.", get_name());
            nav_state_ = NavState::SUCCEEDED;
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "[%s] Goal ABORTED by Nav2.", get_name());
            nav_state_ = NavState::FAILED;
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "[%s] Goal CANCELLED.", get_name());
            nav_state_ = NavState::CANCELLED;
            break;
          default:
            nav_state_ = NavState::FAILED;
        }
        goal_handle_.reset();
        publish_status();
      };

    nav_client_->async_send_goal(goal_msg, send_goal_options);
    nav_state_ = NavState::NAVIGATING;
    publish_status();
    return true;
  }

  NavState get_nav_state() const { return nav_state_; }
  std::string get_robot_ns() const { return robot_ns_; }

private:
  // ── Helpers ─────────────────────────────────────────────────────────────

  void cancel_current_goal()
  {
    if (goal_handle_ && nav_state_ == NavState::NAVIGATING) {
      RCLCPP_INFO(get_logger(), "[%s] Cancelling active goal.", get_name());
      nav_client_->async_cancel_goal(goal_handle_);
      nav_state_ = NavState::CANCELLED;
    }
  }

  void publish_status()
  {
    if (!status_pub_ || !status_pub_->is_activated()) return;
    auto msg = std_msgs::msg::String();
    msg.data = nav_state_str(nav_state_);
    status_pub_->publish(msg);
  }

  // ── Members ──────────────────────────────────────────────────────────────
  std::string  robot_ns_;
  double       goal_timeout_{60.0};
  NavState     nav_state_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr  nav_client_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  GoalHandleNav::SharedPtr goal_handle_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Node name comes from ROS arguments; namespace from parameter
  auto node = std::make_shared<RobotLifecycleNode>("robot_lifecycle_node");

  // Automatically walk through the lifecycle for demo purposes.
  // In production the task_allocator.py drives these transitions.
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());

  // Trigger configure → activate
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  exe.spin_some(500ms);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  exe.spin();
  rclcpp::shutdown();
  return 0;
}
