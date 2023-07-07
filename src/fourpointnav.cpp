#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

static double g_X;
static double g_Y;
float prevDistance = 0.0;
float prevNavigationTime = 0.0;
size_t currNumOfRecoveries = 0;
size_t navtoCheckpoint = 0;
size_t g_totalCheckpoints = 0;

float totalNavigationTime = 0;
size_t totalNumOfRecoveries = 0;
float totalDistanceCovered = 0;

class NavigateToPoseNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit NavigateToPoseNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("send_goal_client" + std::to_string(navtoCheckpoint), node_options), goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "navigate_to_pose");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&NavigateToPoseNode::send_goal, this));
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = NavigateToPose::Goal();

    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.header.frame_id = "map";

    goal_msg.pose.pose.position.x = g_X;
    goal_msg.pose.pose.position.y = g_Y;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal");
    

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&NavigateToPoseNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&NavigateToPoseNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&NavigateToPoseNode::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
      if(prevDistance < feedback->distance_remaining)
      {
          prevDistance = feedback->distance_remaining;
      }
      if(prevNavigationTime < (static_cast<float>(feedback->navigation_time.sec)))
      {
          prevNavigationTime = static_cast<float>(feedback->navigation_time.sec);
      }

      currNumOfRecoveries = feedback->number_of_recoveries;

     RCLCPP_INFO(
      this->get_logger(),
      "\n\nNavigating to Checkpoint: %d\nCurrent Pose : x:%f, y: %f\nDistance Remaining:  %f m\nNumber of Recoveries: %d\nTotal Time Taken: %d.%d s\n" ,
      navtoCheckpoint,
      feedback->current_pose.pose.position.x, 
      feedback->current_pose.pose.position.y, 
      feedback->distance_remaining,
      feedback->number_of_recoveries,
      feedback->navigation_time.sec,
      feedback->navigation_time.nanosec
      );
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Navigation Successful");
  }
};  // class NavigateToPoseNode

int main(int argc, char ** argv)
{


    rclcpp::init(argc, argv);
    ++navtoCheckpoint;

    g_X = 1.0;
    g_Y = 5.50;

    auto cp1 = std::make_shared<NavigateToPoseNode>();
    while (!cp1->is_goal_done()) {
    rclcpp::spin_some(cp1);
    }

    totalDistanceCovered += prevDistance;
    totalNavigationTime += prevNavigationTime;
    totalNumOfRecoveries += currNumOfRecoveries;

    prevDistance = 0.0;
    prevNavigationTime = 0.0;
    currNumOfRecoveries = 0;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ++navtoCheckpoint;
    g_X = 1.0;
    g_Y = -6.50;

    auto cp2 = std::make_shared<NavigateToPoseNode>();
    while (!cp2->is_goal_done()) {
    rclcpp::spin_some(cp2);
    }

    totalDistanceCovered += prevDistance;
    totalNavigationTime += prevNavigationTime;

    prevDistance = 0.0;
    prevNavigationTime = 0.0;
    currNumOfRecoveries = 0;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ++navtoCheckpoint;
    g_X = -4.2;
    g_Y = 1.00;

    auto cp3 = std::make_shared<NavigateToPoseNode>();
    while (!cp3->is_goal_done()) {
    rclcpp::spin_some(cp3);
    }

    totalDistanceCovered += prevDistance;
    totalNavigationTime += prevNavigationTime;

    prevDistance = 0.0;
    prevNavigationTime = 0.0;
    currNumOfRecoveries = 0;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ++navtoCheckpoint;
    g_X = -3.30;
    g_Y = -4.10;

    auto cp4 = std::make_shared<NavigateToPoseNode>();
    while (!cp4->is_goal_done()) {
    rclcpp::spin_some(cp4);
    }    
    
    totalDistanceCovered += prevDistance;
    totalNavigationTime += prevNavigationTime;

    prevDistance = 0.0;
    prevNavigationTime = 0.0;
    currNumOfRecoveries = 0;

    RCLCPP_INFO(cp4->get_logger(),
    "\n\n***Four-Point Navigation Success! :) *** \n\nTotal Distance Covered:  %f m\nTotal Number of Recoveries: %d\nTotal Time Taken: %f s\nTotal Checkpoints Reached: %d \n" , 
    totalDistanceCovered,
    totalNumOfRecoveries,      
    totalNavigationTime,
    g_totalCheckpoints
    );
    std::this_thread::sleep_for(std::chrono::seconds(10));



    rclcpp::shutdown();
    return 0;
}