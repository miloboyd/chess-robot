#include "boardPos.h"
#include "robotControl.h"


//take in paramaters of positiosn and a boolean to represent taken status 
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  /**
  auto const moveit = std::make_shared<rclcpp::Node>(
    "moveit_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)  //alternative
  );
  auto node = std::make_shared<RobotControl>();
  using moveit::planning_interface::MoveGroupInterface;
  executor.add_node(moveit); //alternative
  */

  //create robot control
  auto robot_control = RobotControl::create();

  //Create chessboard with robot dependency
  BoardPos chess_board(robot_control); 
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_control);
  auto spinner = std::thread([&executor]() {executor.spin();});

  //initialise robot position
  robot_control->moveBoard();

  std::this_thread::sleep_for(std::chrono::seconds(2));

  //robot_control->moveLinear(0.15, 0.25, 0.074);
  std::string move = "c8c40";
  chess_board.movePiece(move);

  //finish 
  robot_control->moveHome();


  
  rclcpp::shutdown();
  spinner.join();
  return 0;
}