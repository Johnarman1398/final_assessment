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
size_t g_totalCheckpoints = 0;
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
            createPose(-4.2, 1.0, 0.0, 0.0),
            createPose(-3.30, -4.10, 0.0, 0.0)
        };

        sendGoals(poses);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_action_client_;
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr navigation_goal_handle_;

    float g_feedback_distance = 0.0;
    size_t g_totalCheckpoints = 0;
    size_t g_targetCheckpoint = 0;
    size_t g_feedbackCheckpoint = 0;
    bool g_isNavigating = false;
    bool checkpointReached = false;
    
    

    float totalNavigationTime = 0;
    size_t totalNumOfRecoveries = 0;
    float totalDistanceCovered = 0;

    float prevDistance = 0;
    float prevNavigationTime = 0;
    float currentNavigationTime = 0;

    void sendGoals(const std::vector<geometry_msgs::msg::PoseStamped>& poses)
    {
        for (const auto& pose : poses) {
            ++g_targetCheckpoint;
            bool is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(60));
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
	    
            std::this_thread::sleep_for(std::chrono::seconds(2));

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
        prevDistance = feedback->distance_remaining;
        if(prevDistance < feedback->distance_remaining)
        {
            prevDistance = feedback->distance_remaining;
        }
        if(prevNavigationTime < (static_cast<float>(feedback->navigation_time.sec) + static_cast<float>(feedback->navigation_time.nanosec)/1000000000))
        {
            prevNavigationTime = static_cast<float>(feedback->navigation_time.sec) + static_cast<float>(feedback->navigation_time.nanosec)/1000000000;
        }

        if(checkpointReached)
        {
            totalNavigationTime += prevNavigationTime;
            totalNumOfRecoveries += feedback->number_of_recoveries; 
            totalDistanceCovered += prevDistance;
            checkpointReached = false;
        }

        RCLCPP_INFO(this->get_logger(),
        "\n\nNavigating to Checkpoint: %d \nCurrent Pose : x:%f, y: %f\nDistance Remaining:  %f m\nNumber of Recoveries: %d\nTotal Time Taken: %d.%d s\nTotal Checkpoints Reached: %d" ,      
        g_targetCheckpoint,
        feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y,
        feedback->distance_remaining,
        feedback->number_of_recoveries,      
        feedback->navigation_time.sec, feedback->navigation_time.nanosec,
        g_totalCheckpoints
        );
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result)
    {
        g_isNavigating = false;
        checkpointReached = true;

        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            // RCLCPP_ERROR(this->get_logger(),
            // "\n\n***Four-Point Navigation Failed 1st! :( *** \n\nCurrent Pose : x:%f, y: %f\nDistance Remaining:  %f m\nNumber of Recoveries: %d\nTotal Time Taken: %d.%d s\nTotal Checkpoints Reached: %d \n" ,      
            // this->prevNavigationTime, this->prevNavigationTime,
            // this->prevNavigationTime,
            // g_totalCheckpoints,      
            // this->prevNavigationTime, this->prevNavigationTime,
            // g_totalCheckpoints
            // );
        }
        
        if (g_totalCheckpoints == 4)
        {
            RCLCPP_INFO(this->get_logger(),
            "\n\n***Four-Point Navigation Success! :) *** \n\nTotal Distance Covered:  %f m\nTotal Number of Recoveries: %d\nTotal Time Taken: %d s\nTotal Checkpoints Reached: %d \n" , 
            totalDistanceCovered,
            totalNumOfRecoveries,      
            totalNavigationTime,
            g_totalCheckpoints
            );
        }
        else if ( g_totalCheckpoints < 4 &&  result.code == rclcpp_action::ResultCode::SUCCEEDED) ++g_totalCheckpoints;
        else
        {
            RCLCPP_ERROR(this->get_logger(),
            "\n\n***Four-Point Navigation Failed! :( *** \n\nTotal Distance Covered:  %f m\nTotal Number of Recoveries: %d\nTotal Time Taken: %d s\nTotal Checkpoints Reached: %d \n" , 
            totalDistanceCovered,
            totalNumOfRecoveries,      
            totalNavigationTime,
            g_totalCheckpoints
            );
        }
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
