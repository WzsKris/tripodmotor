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
#include <chrono>
#include <thread>
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
boost::asio::serial_port serial(io, "/dev/ttyACM1");

Semaphore semaphore;


void legcontrol(const std::shared_ptr<core_interfaces::srv::UiMotor::Request> request,
         std::shared_ptr<core_interfaces::srv::UiMotor::Response> response)
{
  // Configure serial ports (if not already configured)
  semaphore.acquire();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Leg1Enterance");
    
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
  //int pos = request->leg1;
  
  int pos = 0;
  
  pos = request->leg1;
  int speed,speed1,speed3;
  int angle,angle1,angle3;
  int m1offset=5;
  int m4offset=50;
  int m7offset=30;

  (void)speed3;
  (void)angle3;


	//For the love of god if you still see this as your state machine please change to a forloop. didnt have time :) <3
	if(pos == 1){



	//Upper
	int speed = 10;
	int angle = 0;

	universialControl(13, 2,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 5,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 8,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	//Lower
	int speed1 = 10;
	int angle1 = -45;


	universialControl(13, 1,angle1+m1offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 4,angle1+m4offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 7,angle1+m7offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	}else if(pos == 2){
	//Upper
	int speed3 = 10;
        int angle3 = 0;
        for(int i = 0; i <= 5; i++) {
		cout << "Command Sent" << endl;
		universialControl(13, 3, angle3, speed3, 0, 0, serial); //M1 144
	}

	
	
	speed = 10;
	angle = -50;

	universialControl(13, 2,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 5,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 8,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	//Lower
	speed1 = 5;
	angle1 = -290;


	universialControl(13, 1,angle1+m1offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 4,angle1+m4offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 7,angle1+m7offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	}else if(pos == 3){
	//Upper
	speed = 30;
	angle = -90;

	universialControl(13, 2,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 5,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 8,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	//Lower
	speed1 = 30;
	angle1 = -270;


	universialControl(13, 1,angle1+m1offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 4,angle1+m4offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 7,angle1+m7offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	}else if(pos == 4){
	//Upper
	speed = 30;
	angle = -120;

	universialControl(13, 2,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 5,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 8,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	//Lower
	speed1 = 30;
	angle1 = -250;


	universialControl(13, 1,angle1+m1offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 4,angle1+m4offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 7,angle1+m7offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	}else if(pos == 5){
	
	
	for(int i; i<=5;i++){
	//Upper
	speed = 30;
	angle = -90;

	universialControl(13, 2,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 5,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 8,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	//Lower
	speed1 = 30;
	angle1 = -270;


	universialControl(13, 1,angle1+m1offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 4,angle1+m4offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 7,angle1+m7offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];
	std::this_thread::sleep_for(std::chrono::seconds(5)), std::cout << "5 seconds have passed!" << std::endl;
	//Upper
	speed = 30;
	angle = -120;

	universialControl(13, 2,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 5,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 8,angle, speed, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	//Lower
	speed1 = 30;
	angle1 = -250;


	universialControl(13, 1,angle1+m1offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 4,angle1+m4offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];

	universialControl(13, 7,angle1+m7offset, speed1, 0, 0, serial); //M1 144
	readResponse(serial, buf, ec)[3];
	
	}
	}
	
  // Log the incoming request
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\n Leg1: %u ", request->leg1);

  // Log the response being sent back
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%u]", response->beat);
  semaphore.release();
}

int main(int argc, char **argv)
{
  // Initialize rclcpp
  rclcpp::init(argc, argv);

  // Create a ROS 2 node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("leg1");

  // Create a service for controlling motors
  rclcpp::Service<core_interfaces::srv::UiMotor>::SharedPtr service =
    node->create_service<core_interfaces::srv::UiMotor>("motorcontrol1", &legcontrol);

  // Log a message indicating that the server is ready to control Leg 1
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to control Leg 1");

  // Spin the node to handle incoming service requests
  rclcpp::spin(node);

  // Shutdown the node
  rclcpp::shutdown();
  return 0;

}

