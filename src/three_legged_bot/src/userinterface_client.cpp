#include "rclcpp/rclcpp.hpp"
#include "core_interfaces/srv/ui_motor.hpp"

#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State: Client *Command Name*");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Commands:    start:  init:  squat:  stand:  10pushup  ");
      return 1;
  }

  //Defines Node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("userinterface_client");
  
  //Defines service that is needed to be connected to: motorcontrol1 
  //-Leg1-
  rclcpp::Client<core_interfaces::srv::UiMotor>::SharedPtr client1 =
    node->create_client<core_interfaces::srv::UiMotor>("motorcontrol1");
  
  
  //Defines Requests
  auto request1 = std::make_shared<core_interfaces::srv::UiMotor::Request>();
 
  
  if(strcmp(argv[1],"start")==0){
  request1->leg1 = 1;
  }else if(strcmp(argv[1],"init")==0){
  request1->leg1 = 2;
  }else if(strcmp(argv[1],"squat")==0){
  request1->leg1 = 3;
  }else if(strcmp(argv[1],"stand")==0){
  request1->leg1 = 4;
  }else if(strcmp(argv[1],"10pushup")==0){
  request1->leg1 = 5;
  }
  
  while (!client1->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service1 not available, waiting again...");
  }
  

  auto result1 = client1->async_send_request(request1);
  
  //Debug
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "async_send_request PASS");
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result1) == rclcpp::FutureReturnCode::SUCCESS) 
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successful ");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}
