#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <fstream>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include <tf2/exceptions.h>
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

using namespace std::chrono_literals;

float g_feedback_distance = 0.0;
bool g_isNavigating = false;

class FourPointNav : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    FourPointNav() : Node("four_point_nav"), totalNavigationTime(0.0)
    {
        navigation_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        

        // List of target poses
        std::vector<geometry_msgs::msg::PoseStamped> poses = {
            createPose(1.0, 5.50, 0.0, 0.0),
            createPose(1, -6.5, 0.0, 0.0),
            createPose(-5.0, 1.0, 0.0, 0.0),
            createPose(-3.30, -4.10, 0.0, 0.0)
        };

        sendGoals(poses);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_action_client_;
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr navigation_goal_handle_;

    float totalNavigationTime;
    float prevNavigationTime;
    float currentNavigationTime;

    void sendGoals(const std::vector<geometry_msgs::msg::PoseStamped>& poses)
    {
        for (const auto& pose : poses) {
            auto is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(100));
            if (!is_action_server_ready) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "navigate_to_pose action server is not available. Is the initial pose set?");
                return;
            }

            NavigateToPose::Goal navigation_goal_;
            navigation_goal_.pose = pose;

            RCLCPP_INFO(this->get_logger(), "NavigateToPose will be called using the BT Navigator's default behavior tree.");

            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

            send_goal_options.goal_response_callback = std::bind(&FourPointNav::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback = std::bind(&FourPointNav::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback = std::bind(&FourPointNav::result_callback, this, std::placeholders::_1);

            navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
            g_isNavigating = true;

            // Wait for the current goal to complete before sending the next one
            while (g_isNavigating) {
                rclcpp::spin_some(this->get_node_base_interface());
            }
        }
    }

    void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> goal_handle)
    {
        if (!goal_handle.get()) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for the result");
        }
    }

    void feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        if(prevNavigationTime > (feedback->navigation_time.sec +feedback->navigation_time.nanosec))
        prevNavigationTime = feedback->navigation_time.sec +feedback->navigation_time.nanosec;

        RCLCPP_INFO(      this->get_logger(),
      "\n\nCurrent Pose : x:%f, y: %f\nDistance Remaining:  %f m\nNumber of Recoveries: %d\nTotal Time Taken: %d.%d s\n" ,      
      feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y,
      feedback->distance_remaining,
      feedback->number_of_recoveries,      
      feedback->navigation_time.sec, feedback->navigation_time.nanosec      
      );
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded");
        } else {
            RCLCPP_INFO(this->get_logger(), "Navigation goal failed");
        }
        g_isNavigating = false;

        RCLCPP_INFO(this->get_logger(), "%f", this->prevNavigationTime);
    }

    geometry_msgs::msg::PoseStamped createPose(double x, double y, double z, double orientation)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = rclcpp::Clock().now();
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(orientation);
        return pose;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FourPointNav>());
    rclcpp::shutdown();
    return 0;
}
