#include "boardPos.h"
#include "robotControl.h"
#include <std_msgs/msg/bool.hpp>



//take in paramaters of positiosn and a boolean to represent taken status 
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  //auto chess_node = rclcpp::Node::make_shared("chess_main");
  //bool start_received = false;
  //bool robot_turn = false;


  //wait for robot turn 
  
  //create robot control
  auto robot_control = RobotControl::create();
  BoardPos chess_board(robot_control); 

  rclcpp::executors::MultiThreadedExecutor executor; //perhaps single threaded is better 
  executor.add_node(robot_control);
  //executor.add_node(chess_node);
  auto spinner = std::thread([&executor]() {executor.spin();});

  //initialise robot position
  robot_control->moveBoard();

  std::this_thread::sleep_for(std::chrono::seconds(2));

  //robot_control->moveLinear(0.15, 0.25, 0.074);
  std::string move = "e2e41";
  chess_board.movePiece(move);
  //robot_control->getPos();
  //std::string newMove = "h8h10";
  //chess_board.movePiece(newMove);



  //finish 
  robot_control->moveHome();


  
  rclcpp::shutdown();
  spinner.join();
  return 0;
}