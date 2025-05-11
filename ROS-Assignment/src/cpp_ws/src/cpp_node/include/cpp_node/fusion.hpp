#include <memory>
#include <string>
#include <chrono>
#include <unordered_map>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "custom_msgs/action/map_navigation.hpp"
using namespace std::chrono_literals;
constexpr auto _1 = std::placeholders::_1;
constexpr auto _2 = std::placeholders::_2;

class MultiMapNavigator : public rclcpp::Node {
public:
  MultiMapNavigator();  

private:

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using NavigateToRoom = custom_msgs::action::MapNavigation;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToRoom>;
rclcpp_action::Server<custom_msgs::action::MapNavigation>::SharedPtr nav_to_room_action_server_;



  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> wormholes_;
  std::string current_map_;
  std::string map_to_switch;
  //GoalHandleNav::SharedPtr nav_to_pose_goal_handle_;
  std::shared_ptr<GoalHandleNav> nav_to_pose_goal_handle_;
  std::shared_ptr<GoalHandle> goal_handle_;
  NavigateToRoom::Result result_;

  std::mutex mutex_;
  bool awaiting_wormhole_nav_; 
  bool result_called_;
  std::string status_;
  geometry_msgs::msg::PoseStamped target_after_map_switch_;


  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToRoom::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

  

  void execute(
    const std::shared_ptr<GoalHandle> goal_handle);
  void navigate_to_pose(const geometry_msgs::msg::PoseStamped & target_pose);

    template <typename ActionT>                                                           
    void clientResponseCallback(const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr &goal_handle);
     //Template functions for Result, Response and Feedback callbacks for the 3 action clients
    template <typename ActionT>
    void clientResultCallback(const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult &result);

    template <typename ActionT>
    void clientFeedbackCallback(const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr &goal_handle,
                                  const std::shared_ptr<const typename ActionT::Feedback> feedback);
    bool switch_map(const std::string & map_name);
   
    geometry_msgs::msg::PoseStamped create_pose(double x, double y, const std::string& frame_id);

    
    bool navigate_to(const geometry_msgs::msg::PoseStamped & target_pose);  // assumed
    void PathNavResult(std::string _result);

    //std::shared_ptr<GoalHandleNav> goal_handle_;  // Add this line
    //std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle_;



    
    
};
