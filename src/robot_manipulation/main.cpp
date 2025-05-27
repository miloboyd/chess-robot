#include "main.h"
#include "boardPos.h"
#include "robotControl.h"

// //take in paramaters of positiosn and a boolean to represent taken status 
// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);

//   //create robot control
//   auto robot_control = RobotControl::create();

//   //Create chessboard with robot dependency
//   BoardPos chess_board(robot_control); 
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(robot_control);
//   auto spinner = std::thread([&executor]() {executor.spin();});

//   //initialise robot position
//   robot_control->moveBoard();

//   std::this_thread::sleep_for(std::chrono::seconds(2));

//   //robot_control->moveLinear(0.15, 0.25, 0.074);
//   std::string move = "c8c40";
//   chess_board.movePiece(move);

//   //finish 
//   robot_control->moveHome();


  
//   rclcpp::shutdown();
//   spinner.join();
//   return 0;
// }


Main::Main() : Node("chess_robot_node") {
  RCLCPP_INFO(this->get_logger(), "Initializing Chess Robot Node");

  //robot_control_ = std::make_unique<RobotControl>(shared_from_this());
  board_pos_ = std::make_unique<BoardPos>();

  //create service
  start_game_service_ = this->create_service<std_srvs::srv::Trigger>(
    "ur3/start_signal", 
    std::bind(&Main::startGameCallback, this, 
             std::placeholders::_1, std::placeholders::_2)
  );

  //create subscriber 
  ai_move_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/send_move", 10,
      std::bind(&Main::aiMoveCallback, this, std::placeholders::_1)
  );

  // Create publisher
  move_complete_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "/move_complete", 10);
  
  RCLCPP_INFO(this->get_logger(), "Chess Robot Node initialized successfully");

}

void Main::run() {
  // Initialize robot (separate from constructor to handle MoveIt timing issues)

  robot_control_ = std::make_unique<RobotControl>(shared_from_this());
  RCLCPP_INFO(this->get_logger(), "Creating robot controller...");

  if (!initialiseRobot()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize robot");
      return;
  }
  
  // Create GUI in main thread (Qt requirement)
  RCLCPP_INFO(this->get_logger(), "Creating GUI in main thread");
  GUI_ = std::make_unique<GUI>(shared_from_this());
  GUI_->show();
  RCLCPP_INFO(this->get_logger(), "GUI shown");
  
  RCLCPP_INFO(this->get_logger(), "Chess Robot ready - waiting for game start service call");
  
  // Use MultiThreadedExecutor to handle ROS2 callbacks in background threads
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shared_from_this());
  
  // Spin ROS2 in separate thread so it doesn't block Qt
  std::thread ros_thread([&executor]() {
      RCLCPP_INFO(rclcpp::get_logger("ros_thread"), "Starting ROS2 executor spin");
      executor.spin();
      RCLCPP_INFO(rclcpp::get_logger("ros_thread"), "ROS2 executor spin finished");
  });
  
  // Run Qt event loop in main thread - this blocks until GUI is closed
  RCLCPP_INFO(this->get_logger(), "Starting Qt event loop in main thread");
  int qt_result = QApplication::exec();
  RCLCPP_INFO(this->get_logger(), "Qt event loop finished with code: %d", qt_result);
  
  // When Qt exits, shutdown ROS2
  RCLCPP_INFO(this->get_logger(), "Shutting down ROS2");
  rclcpp::shutdown();
  
  // Wait for ROS2 thread to finish
  if (ros_thread.joinable()) {
      ros_thread.join();
  }
  
  RCLCPP_INFO(this->get_logger(), "Application shutdown complete");
}

bool Main::initialiseRobot() {
  RCLCPP_INFO(this->get_logger(), "Initializing robot controller...");
  
  // Allow time for node to be fully established (fixes your MoveIt timing issue)
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  if (!robot_control_->initialise()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize robot controller");
      return false;
  }
  
  return true;
}

// Service callback implementation
void Main::startGameCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  
  RCLCPP_INFO(this->get_logger(), "Game start requested");

  // Start the game
  game_active_ = true;
  response->success = true;
  response->message = "Game started successfully";
  
  RCLCPP_INFO(this->get_logger(), "Game started successfully");
}

// Subscriber callback implementations
void Main::aiMoveCallback(const std_msgs::msg::String::SharedPtr msg) {

  if (!game_active_) {
    RCLCPP_WARN(this->get_logger(), "Received move but game not started");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received move: %s", msg->data.c_str());
  
  // Execute move in separate thread to not block ROS2 callbacks
  std::thread([this, move = msg->data]() {
      robot_control_->moveBoard();
      bool success = executeMove(move);
      //robot_control_->moveHome();
      publishMoveComplete(false);
      
      if (success) {
          RCLCPP_INFO(this->get_logger(), "Move executed successfully: %s", move.c_str());
      } else {
          RCLCPP_ERROR(this->get_logger(), "Move execution failed: %s", move.c_str());
      }
      
      // Reset for next turn
  }).detach();
}

// Move execution implementation
bool Main::executeMove(const std::string& notation) {
  RCLCPP_INFO(this->get_logger(), "Executing move: %s", notation.c_str());

  std::vector<int> result = board_pos_->chessNotationToIndex(notation);
  if (result.empty()) {
      std::cerr << "Failed to parse chess notation" << std::endl;
      return false;
  }
  
  int firstPos = result[0];
  int secondPos = result[1];
  bool occupied = result[2];

  auto capturedPos = board_pos_->getCapturedPiecePosition();
  auto startPos = board_pos_->getBoardPosition(firstPos); 
  auto opponentPos = board_pos_->getBoardPosition(secondPos);

  robot_control_->moveBoard();
  
  // Execute the planned move sequence
  if (occupied) {

    //move opponent piece to black captured piece
    std::cout << "Moving captured white piece to white captured board" << std::endl;
    


    //proceed with movement 
    robot_control_->moveLinear(opponentPos.x,opponentPos.y,opponentPos.z);
    robot_control_->pickUpPiece();
    robot_control_->moveLinear(capturedPos.x,capturedPos.y,capturedPos.z);
    robot_control_->placePiece();
}

//Check for pawn promotion
if (board_pos_->isPawnPromotion(secondPos)) {
    // handlePawnPromotion(destination);
    std::cout << "Pawn promotion detected! (Implementation pending)" << std::endl;
    //when pulling piece, remove taken piece type 
}

//move main piece 
robot_control_->moveLinear(startPos.x,startPos.y,startPos.z);
robot_control_->pickUpPiece();
robot_control_->moveLinear(opponentPos.x,opponentPos.y,opponentPos.z);
robot_control_->placePiece();


robot_control_->moveHome();

return true;
}

// Utility methods
void Main::publishMoveComplete(bool success) {
  auto msg = std_msgs::msg::Bool();
  msg.data = success;
  move_complete_publisher_->publish(msg);
}
