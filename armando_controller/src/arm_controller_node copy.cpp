// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
// #include "trajectory_msgs/msg/JointTrajectory.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ArmandoController : public rclcpp::Node
{
public:
  ArmandoController(int ctrl=0)
  : Node("arm_controller_node"), count_(0)
  {
    if(ctrl==0)
      publisher_pos = this->create_publisher<std_msgs::msg::Float64MultiArray>("position_controller/commands", 10);
    else 
      publisher_traj = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_trajectory_controller/joint_trajectory", 10);
    
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
       "joint_states", 10, std::bind(&ArmandoController::topic_callback, this, _1));

    // timer_ = this->create_wall_timer(
    //  500ms, std::bind(&ArmandoController::timer_callback, this));
  }

private:
  // void topic_pos_controller(const int numero_giunti, const sensor_msgs::msg::JointState & msg)const{

  // }
  void topic_callback(const sensor_msgs::msg::JointState & msg) const
  {
    int n = msg.name.size(); // numero di giunti

    for(int i=0; i<n; i++)
      RCLCPP_INFO(this->get_logger(), "I heard joint: '%s'\tpos=%f\tvel=%f\teff=%f", 
                  msg.name[i].c_str(), msg.position[i], msg.velocity[i], msg.effort[i]);

    // Publisher position controller:
    auto pos_command = std_msgs::msg::Float64MultiArray();
    static int ref=0; // switch case

    if(ref==0){
      // riferimento 0
      pos_command.data= {0.0, 0.75, -0.75, 0.0};  // esempio di posizioni target
      RCLCPP_INFO(this->get_logger(), "Publishing command position %d: {%f, %f, %f, %f}", 
                  ref,
                  pos_command.data[0], pos_command.data[1], pos_command.data[2], pos_command.data[3]);
      publisher_pos->publish(pos_command);


      float* position_error = new float[msg.name.size()];
      int count_zero = 0;
      float threshold = 0.00001;
      for(int i=0; i<n; i++){
        position_error[i] = pos_command.data[i] - msg.position[i];
        printf("\n%f",position_error[i]);
        if(abs(position_error[i])<threshold){
          count_zero++;
        }
      }   
      if(count_zero==4) ref++; 

      delete[] position_error; // memory leak uagliu
    }
    if(ref==1){
      // riferimento 1:
      pos_command.data = {0.0, -0.75, 0.75, 0.0};  // posizioni target
      RCLCPP_INFO(this->get_logger(), "Publishing command position %d: {%f, %f, %f, %f}", 
                  ref,
                  pos_command.data[0], pos_command.data[1], pos_command.data[2], pos_command.data[3]);
      publisher_pos->publish(pos_command);


      float* position_error = new float[msg.name.size()];
      int count_zero = 0;
      float threshold = 0.00001;
      for(int i=0; i<n; i++){
        position_error[i] = pos_command.data[i] - msg.position[i];
        printf("\n%f",position_error[i]);
        if(abs(position_error[i])<threshold){
          count_zero++;
        }
      }   
      if(count_zero==4) ref++; 

      delete[] position_error; // memory leak uagliu
    }
    if(ref==2){
      // riferimento 1:
      pos_command.data = {0.0, -0.75, 0.75, -0.5};  // posizioni target
      RCLCPP_INFO(this->get_logger(), "Publishing command position %d: {%f, %f, %f, %f}", 
                  ref,
                  pos_command.data[0], pos_command.data[1], pos_command.data[2], pos_command.data[3]);
      publisher_pos->publish(pos_command);


      float* position_error = new float[msg.name.size()];
      int count_zero = 0;
      float threshold = 0.00001;
      for(int i=0; i<n; i++){
        position_error[i] = pos_command.data[i] - msg.position[i];
        printf("\n%f",position_error[i]);
        if(abs(position_error[i])<threshold){
          count_zero++;
        }
      }   
      if(count_zero==4) ref++; 

      delete[] position_error; // memory leak uagliu
    }
    if(ref==3){
      // riferimento 1:
      pos_command.data = {0.0, -0.75, 0.75, 0.5};  // posizioni target
      RCLCPP_INFO(this->get_logger(), "Publishing command position %d: {%f, %f, %f, %f}", 
                  ref,
                  pos_command.data[0], pos_command.data[1], pos_command.data[2], pos_command.data[3]);
      publisher_pos->publish(pos_command);


      float* position_error = new float[msg.name.size()];
      int count_zero = 0;
      float threshold = 0.00001;
      for(int i=0; i<n; i++){
        position_error[i] = pos_command.data[i] - msg.position[i];
        printf("\n%f",position_error[i]);
        if(abs(position_error[i])<threshold){
          count_zero++;
        }
      }   
      if(count_zero==4) ref++; 

      delete[] position_error; // memory leak uagliu
    }

  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_pos;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_traj;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  size_t count_;
  int ctrl;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // debug
  std::cout << "[DEBUG] argc = " << argc << std::endl;
  for (int i = 0; i < argc; i++)
      std::cout << "[DEBUG] argv[" << i << "] = " << argv[i] << std::endl;
  // conversione a intero
  int sel = argv[1][0]-48;
  //start spin
  rclcpp::spin(std::make_shared<ArmandoController>(sel));
  rclcpp::shutdown();
  
  return 0;
}
