//Standard Libarires
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdint>
#include <chrono>
#include <cstdlib>
#include <memory>
//External Libraries
#include <iomanip>
#include <boost/asio.hpp>


//ROS2 HPP files
#include "rclcpp/rclcpp.hpp"
#include "core_interfaces/srv/ui_motor.hpp"

//Header files
#include "Core.h"
#include "semaphore.h"

using namespace std;
using namespace boost::asio;
using boost::system::error_code;
using namespace std::chrono;


// Define the io_context and serial_port objects globally
boost::asio::io_context io;
boost::asio::serial_port serial(io, "/dev/ttyACM0");


Semaphore semaphore;


void legcontrol(const std::shared_ptr<core_interfaces::srv::UiMotor::Request> request,
         std::shared_ptr<core_interfaces::srv::UiMotor::Response> response)
{
  semaphore.acquire();
  // Configure serial ports (if not already configured)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Leg3Enterance");
    //serial.open("/dev/ttyACM0");
    serial.set_option(serial_port_base::baud_rate(115200));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
  

  //Buffer vector used to read incoming data.
  std::vector<uint8_t> buf(128); // Adjust the size based on expected data in Bytes.
  boost::system::error_code ec;

  // Calculate the sum of elements in the arrays
  response->beat = request->leg1;

  // Call universialControl and readResponse functions
  universialControl(13, 7, request->leg1, 50, 0, 0, serial);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CommandSentl2");
  readResponse(serial, buf, ec);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rxl2");
 
  // Log the incoming request
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\n Leg3: %u ", request->leg1);

  // Log the response being sent back
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%u]", response->beat);
  semaphore.release();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "leg3 released");
}

int main(int argc, char **argv)
{
  // Initialize rclcpp
  rclcpp::init(argc, argv);

  // Create a ROS 2 node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("leg3");

  // Create a service for controlling motors
  rclcpp::Service<core_interfaces::srv::UiMotor>::SharedPtr service =
    node->create_service<core_interfaces::srv::UiMotor>("motorcontrol3", &legcontrol);

  // Log a message indicating that the server is ready to control Leg 1
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to control Leg 3");

  // Spin the node to handle incoming service requests
  rclcpp::spin(node);

  // Shutdown the node
  rclcpp::shutdown();
  return 0;
}
