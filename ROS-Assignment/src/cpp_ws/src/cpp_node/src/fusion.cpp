#include "cpp_node/fusion.hpp"


  MultiMapNavigator::MultiMapNavigator()
    : Node("multi_map_navigator"),awaiting_wormhole_nav_(false),result_called_(false) {
  
    nav_to_room_action_server_ = rclcpp_action::create_server<custom_msgs::action::MapNavigation>(
      this,
      "navigate_to_room",
      std::bind(&MultiMapNavigator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MultiMapNavigator::handle_cancel, this, std::placeholders::_1),
      std::bind(&MultiMapNavigator::handle_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options());

    load_map_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

    nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");

    // Wormhole poses
    wormholes_["map1_to_map2"] = create_pose(11.366560935974121, -6.855052471160889, "map");
    wormholes_["map2_to_map1"] = create_pose(11.366560935974121, -6.855052471160889, "map");
    current_map_ = "map1";
   
  
  }


 

  bool MultiMapNavigator::switch_map(const std::string & map_name) {
  if (!load_map_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(), "LoadMap service not available");
    return false;
  }
 


  // Map names to file paths
  std::unordered_map<std::string, std::string> map_paths = {
    {"map1", "/home/thedush/Videos/Food-Delivery-Bot/ROS-Assignment/src/butlerbot_localization/maps/map1.yaml"},
    {"map2", "/home/thedush/Videos/Food-Delivery-Bot/ROS-Assignment/src/butlerbot_localization/maps/map2.yaml"}
  };

  if (map_paths.find(map_name) == map_paths.end()) {
    RCLCPP_ERROR(this->get_logger(), "Unknown map name: %s", map_name.c_str());
    return false;
  }

  auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  request->map_url = map_paths[map_name];

  auto future = load_map_client_->async_send_request(request);


  std::this_thread::sleep_for(std::chrono::seconds(5));



  RCLCPP_INFO(this->get_logger(), "Switched to map: %s", map_name.c_str());
  return true;
}


  geometry_msgs::msg::PoseStamped MultiMapNavigator::create_pose(double x, double y, const std::string & frame_id) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  rclcpp_action::GoalResponse MultiMapNavigator::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToRoom::Goal> goal)
  
  {
    RCLCPP_INFO(this->get_logger(), "Received goal for map: %s", goal->target_map.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MultiMapNavigator::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MultiMapNavigator::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
  execute(goal_handle);
  }

void MultiMapNavigator::execute(const std::shared_ptr<GoalHandle> goal_handle) {
   goal_handle_.reset();
    if (goal_handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Received null goal_handle.");
    }
  result_called_ = false;
  const auto goal = goal_handle->get_goal();
  map_to_switch = goal->target_map;
  
  RCLCPP_INFO(this->get_logger(), "Goal Received: %s", goal->target_map.c_str());
  const auto& pose = goal->target_pose.pose;

  RCLCPP_INFO(this->get_logger(), "Goal Position -> x: %.2f, y: %.2f, z: %.2f",
              pose.position.x, pose.position.y, pose.position.z);

  RCLCPP_INFO(this->get_logger(), "Goal Orientation -> x: %.2f, y: %.2f, z: %.2f, w: %.2f",
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

  auto feedback = std::make_shared<NavigateToRoom::Feedback>();

  if (goal->target_map == current_map_) {
    RCLCPP_INFO(this->get_logger(), "Target map is current map. Navigating directly.");
    navigate_to_pose(goal->target_pose);
  } else {
    std::string wormhole_key = current_map_ + "_to_" + goal->target_map;
    if (wormholes_.count(wormhole_key) == 0) {
    RCLCPP_ERROR(this->get_logger(), "No wormhole from %s to %s", current_map_.c_str(), goal->target_map.c_str());

    auto result = std::make_shared<NavigateToRoom::Result>();
    result->result = false;
    goal_handle->abort(result);
    return;
    }
    target_after_map_switch_ = goal->target_pose;
    awaiting_wormhole_nav_ = true;
    RCLCPP_INFO(this->get_logger(), "Navigating to wormhole to reach map: %s", goal->target_map.c_str());
    navigate_to_pose(wormholes_[wormhole_key]);
  
}
}




void MultiMapNavigator::navigate_to_pose(const geometry_msgs::msg::PoseStamped & target_pose)
  {    
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = target_pose;
    nav_to_pose_client_ -> wait_for_action_server();
    auto send_pose = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_pose.result_callback = std::bind(& MultiMapNavigator::clientResultCallback<NavigateToPose>, this, _1);
    send_pose.goal_response_callback = std::bind(& MultiMapNavigator::clientResponseCallback<NavigateToPose>, this, _1);
    send_pose.feedback_callback = std::bind(& MultiMapNavigator::clientFeedbackCallback<NavigateToPose>, this, _1, _2);

    //RCLCPP_INFO(this->get_logger(), "Getting path to the nearest edge in the path");
    
    // Send goal asynchronously
    nav_to_pose_client_->async_send_goal(goal_msg, send_pose);
}

//This is the Common Goal response callback for all 3 action clients
template <typename ActionT>
void MultiMapNavigator::clientResponseCallback([[maybe_unused]]
    const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr &goal_handle) {

    std::lock_guard<std::mutex> lock(mutex_);


    if constexpr(std::is_same<ActionT, nav2_msgs::action::NavigateToPose>::value){

        //RCLCPP_INFO(this->get_logger(), "Response received from Compute and Track action server");
        if(goal_handle){
            nav_to_pose_goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Saved Nav To Pose goal handle");
        }
    }
};

//This is the Common Feedback response callback for all 3 action clients
template <typename ActionT>
void  MultiMapNavigator::clientFeedbackCallback([[maybe_unused]]
    const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr &goal_handle,
    const std::shared_ptr<const typename ActionT::Feedback> feedback) {
    
    std::lock_guard<std::mutex> lock(mutex_);
   
    if constexpr(std::is_same<ActionT, nav2_msgs::action::NavigateToPose>::value){
        const auto& pose = feedback->current_pose.pose;
        RCLCPP_INFO(this->get_logger(), 
            "Nav Feedback:\n  - Current Position: [%.2f, %.2f, %.2f]\n  - Distance Remaining: %.2f\n  - Estimated Time Left: %ld.%09lds\n  - Number of Recoveries: %d",
            pose.position.x,
            pose.position.y,
            pose.position.z,
            feedback->distance_remaining,
            feedback->estimated_time_remaining.sec,
            feedback->estimated_time_remaining.nanosec,
            feedback->number_of_recoveries
        );
    
    }
}
template<typename ActionT>
void MultiMapNavigator::clientResultCallback(
    const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult &result)
{
    if constexpr (std::is_same<ActionT, nav2_msgs::action::NavigateToPose>::value) {
        RCLCPP_INFO(this->get_logger(), "Received result for NavToPose");

        result_ = custom_msgs::action::MapNavigation::Result();  // or initialize if needed

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
                if (awaiting_wormhole_nav_){
                  awaiting_wormhole_nav_ = false;
                  bool map_switched = switch_map(map_to_switch);
                  if (map_switched){
                  navigate_to_pose(target_after_map_switch_);
                   }
                   
                }
                else{
                status_ = "Success";
                PathNavResult(status_);
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "Navigation aborted");
                status_ = "Aborted";
                PathNavResult(status_);
               
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigation canceled");
                status_ = "Cancel";
                PathNavResult(status_);
           
                break;
        
        }
    
        // Process the result (publish it, store it, log it, etc.)
        // Do not call `succeed`, `abort`, or `canceled` in a client callback
    }
}

void MultiMapNavigator::PathNavResult(std::string _result){

    //RCLCPP_INFO(this->get_logger(), "Calling Result Function!!!!!!!!!!!!!!!!!!!!!");
    if(goal_handle_){
        if(!result_called_){
            RCLCPP_INFO(this->get_logger(), "Sending the Result to the action server");

            if(_result == "Aborted"){
                result_.outcome = "Aborted";
                result_.result = false;
                auto result_ptr = std::make_shared<NavigateToRoom::Result>(result_);
                goal_handle_->abort(result_ptr);

             
            }
        
            else if (_result == "Success")
            {
                result_.outcome = "Success";
                result_.result = true;
                auto result_ptr = std::make_shared<NavigateToRoom::Result>(result_);
                goal_handle_->succeed(result_ptr);
            }
        
            else if(_result == "Cancel"){
                result_.outcome = "Cancelled";
                result_.result = false;
                auto result_ptr = std::make_shared<NavigateToRoom::Result>(result_);
                goal_handle_->canceled(result_ptr);
            }
            result_called_ = true;

        }
          
    }
}

// template <typename ActionT>
// void MultiMapNavigator::clientResultCallback(
//     const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult &result) {

//   std::lock_guard<std::mutex> lock(mutex_);

//   if constexpr (std::is_same<ActionT, nav2_msgs::action::NavigateToPose>::value) {
//         // Specific logic for Compute and Track Action Server
//         if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//             RCLCPP_INFO(this->get_logger(), "Succeeded - NavToPose");

          
       
//           auto result_ptr = std::make_shared<NavigateToRoom::Result>(result_);
//           result->result = true;
//           result->outcome = "Success";

//           GoalHandle->succeed(result_ptr);
          
//             }

           
//         }
//         else if (result.code == rclcpp_action::ResultCode::ABORTED) {
//             RCLCPP_ERROR(this->get_logger(), " NavToPose action server aborted");
//             auto error_code = result.result->error_code;
//             RCLCPP_ERROR(this->get_logger(),"The error code returned by compute and track action server is %d", error_code);
//             auto result_ptr = std::make_shared<NavigateToRoom::Result>(result_);
//             result_->outcome = "Aborted";
//             result_->result = false;
          
//             GoalHandle->abort(result_ptr);
//         }
//         else if (result.code == rclcpp_action::ResultCode::CANCELED) {
//             RCLCPP_WARN(this->get_logger(), "Cancelled the goal sent to NavToPose action server");
//             auto result_ptr = std::make_shared<NavigateToRoom::Result>(result_);
//             result_->outcome = "Cancelled";
//             result_->result = false;
       
//             GoalHandle->canceled(result_ptr);
    
//         }
//     }




 



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiMapNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


