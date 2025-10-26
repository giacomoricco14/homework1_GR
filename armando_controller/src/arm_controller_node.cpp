// Armando controller node: receiving the JointStates and trasmitting the JointPositions

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the subscription. */

class ArmandoController : public rclcpp::Node
{
  public:
    ArmandoController(int ctrl=0): Node("arm_controller_node"), count_(0), ctrl(ctrl)
    {
      if(ctrl==0)
        publisher_pos = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                      "position_controller/commands", 10);
      else{
        publisher_traj = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                      "joint_trajectory_controller/joint_trajectory", 10);
      }
      
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                      "joint_states", 10, std::bind(&ArmandoController::topic_callback, this, _1));
    }

  private:
    bool all_errors_zero(const int n, 
      const std_msgs::msg::Float64MultiArray &pos_command,
      const sensor_msgs::msg::JointState & msg)const{
      /* Computing the position error */
      float* position_error = new float[msg.name.size()];
      int count_zero = 0;
      float threshold = 0.00001;

      for(int i=0; i<n; i++){
        position_error[i] = pos_command.data[i] - msg.position[i];
        // printf("\n%f",position_error[i]);
        if(abs(position_error[i])<threshold){
          count_zero++;
        }
      }   

      delete[] position_error; // avoiding memory leak

      return count_zero==4;
    } // end all_errors_zero

    void topic_pos_controller(const int n, const sensor_msgs::msg::JointState & msg)const{
      // Publisher position controller:
      auto pos_command = std_msgs::msg::Float64MultiArray();
      static int ref=0; // switch case

      if(ref==0){
        /* Target Joint Positions */
        pos_command.data= {0.0, 0.75, -0.75, 0.0};
        RCLCPP_INFO(this->get_logger(), "[position controller] Publishing command position %d: {%f, %f, %f, %f}", 
                    ref,
                    pos_command.data[0], pos_command.data[1], pos_command.data[2], pos_command.data[3]);
        // Publish the positions
        publisher_pos->publish(pos_command);

        if(all_errors_zero(n, pos_command, msg)) ref++; // go to next reference

      } // end if
      if(ref==1){
        /* Target Joint Positions */
        pos_command.data = {0.0, -0.75, 0.75, 0.0};
        RCLCPP_INFO(this->get_logger(), "[position controller] Publishing command position %d: {%f, %f, %f, %f}", 
                    ref,
                    pos_command.data[0], pos_command.data[1], pos_command.data[2], pos_command.data[3]);
        // Publish the positions
        publisher_pos->publish(pos_command);

        
        if(all_errors_zero(n, pos_command, msg)) ref++; // go to next reference

      } // end if
      if(ref==2){
        /* Target Joint Positions */
        pos_command.data = {0.0, -0.75, 0.75, -0.5};
        RCLCPP_INFO(this->get_logger(), "[position controller] Publishing command position %d: {%f, %f, %f, %f}", 
                    ref,
                    pos_command.data[0], pos_command.data[1], pos_command.data[2], pos_command.data[3]);
        // Publish the positions
        publisher_pos->publish(pos_command);

        if(all_errors_zero(n, pos_command, msg)) ref++; // go to next reference

      } // end if
      if(ref==3){
        /* Target Joint Positions */
        pos_command.data = {0.0, -0.75, 0.75, 0.5};
        RCLCPP_INFO(this->get_logger(), "[position controller] Publishing command position %d: {%f, %f, %f, %f}", 
                    ref,
                    pos_command.data[0], pos_command.data[1], pos_command.data[2], pos_command.data[3]);
        // Publish the positions
        publisher_pos->publish(pos_command);

        if(all_errors_zero(n, pos_command, msg)) ref++; // go to next reference

      } // end if
      
    } // end topic_pos_controller

    void topic_traj_controller(const int n, const sensor_msgs::msg::JointState & msg)const{
      // Publisher trajectory controller:
      auto traj_command = trajectory_msgs::msg::JointTrajectory();
      traj_command.joint_names = {"j0", "j1", "j2", "j3"};
      trajectory_msgs::msg::JointTrajectoryPoint point;
      static int ref=0; // switch case
      
      if(ref==0){
        /* Target Joint Positions */
        point.positions = {0.0, 1.57, 0.0, 0.0};
        // point.velocities = {0.0, 0.0, 0.0, 0.0};
        // point.time_from_start = rclcpp::Duration::from_seconds(5.0);  // settling time

        // Push this point to the points field
        traj_command.points.push_back(point);

        RCLCPP_INFO(this->get_logger(), "[trajectory controller] Publishing command position %d: {%f, %f, %f, %f}", 
                    ref,
                    point.positions[0], point.positions[1], point.positions[2], point.positions[3]);
        // Publish the positions
        publisher_traj->publish(traj_command);

        if(all_errors_zero(n, pos_command, msg)) ref++; // go to next reference

      } // end if
      if(ref==1){
        /* Target Joint Positions */
        point.positions = {0.0, 1.57, 0.35, 0.75};
        // point.velocities = {0.0, 0.0, 0.0, 0.0};
        // point.time_from_start = rclcpp::Duration::from_seconds(5.0);  // settling time

        // Push this point to the points field
        traj_command.points.push_back(point);

        RCLCPP_INFO(this->get_logger(), "[trajectory controller] Publishing command position %d: {%f, %f, %f, %f}", 
                    ref,
                    point.positions[0], point.positions[1], point.positions[2], point.positions[3]);
        // Publish the positions
        publisher_traj->publish(traj_command);

        if(all_errors_zero(n, pos_command, msg)) ref++; // go to next reference

      } // end if
      if(ref==2){
        /* Target Joint Positions */
        point.positions = {0.0, 1.57, -0.75, 0.0};
        // point.velocities = {0.0, 0.0, 0.0, 0.0};
        // point.time_from_start = rclcpp::Duration::from_seconds(5.0);  // settling time

        // Push this point to the points field
        traj_command.points.push_back(point);

        RCLCPP_INFO(this->get_logger(), "[trajectory controller] Publishing command position %d: {%f, %f, %f, %f}", 
                    ref,
                    point.positions[0], point.positions[1], point.positions[2], point.positions[3]);
        // Publish the positions
        publisher_traj->publish(traj_command);

        if(all_errors_zero(n, pos_command, msg)) ref++; // go to next reference

      } // end if
      if(ref==3){
        /* Target Joint Positions */
        point.positions = {0.0, 1.57, 0.0, 0.0};
        // point.velocities = {0.0, 0.0, 0.0, 0.0};
        // point.time_from_start = rclcpp::Duration::from_seconds(5.0);  // settling time

        // Push this point to the points field
        traj_command.points.push_back(point);

        RCLCPP_INFO(this->get_logger(), "[trajectory controller] Publishing command position %d: {%f, %f, %f, %f}", 
                    ref,
                    point.positions[0], point.positions[1], point.positions[2], point.positions[3]);
        // Publish the positions
        publisher_traj->publish(traj_command);

        if(all_errors_zero(n, pos_command, msg)) ref++; // go to next reference

      } // end if

    } // end topic_traj_controller

    void topic_callback(const sensor_msgs::msg::JointState & msg) const
    {
      int n = msg.name.size(); // joint number

      for(int i=0; i<n; i++)
        RCLCPP_INFO(this->get_logger(), "[Subscriber] I heard joint '%s':\tpos=%f\tvel=%f\teff=%f", 
                    msg.name[i].c_str(), msg.position[i], msg.velocity[i], msg.effort[i]);

      if(ctrl==0){
        topic_pos_controller(n, msg);
      }
      else if(ctrl==1){
        topic_traj_controller(n, msg);
      }

    } // end topic_callback


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_pos;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_traj;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    size_t count_;
    int ctrl;
}; // end class


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // debug:
  std::cout << "[DEBUG] argc = " << argc << std::endl;
  for (int i = 0; i < argc; i++)
      std::cout << "[DEBUG] argv[" << i << "] = " << argv[i] << std::endl;
  // conversione a intero
  int sel = argv[1][0]-48;
  std::cout << "[DEBUG] ctrl = " << sel << std::endl;
  //start spin
  rclcpp::spin(std::make_shared<ArmandoController>(sel));
  rclcpp::shutdown();
  
  return 0;
}
