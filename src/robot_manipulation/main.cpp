#include "main.h"
#include "boardPos.h"
#include "robotControl.h"


//Creating a ROS 2 node with the name "chess_robot_node"
Main::Main() : Node("chess_robot_node") {
  RCLCPP_INFO(this->get_logger(), "Initializing Chess Robot Node");

  //robot_control_ = std::make_unique<RobotControl>(shared_from_this());
  board_pos_ = std::make_unique<BoardPos>();

  //define service server that listens for a request to start the game
  start_game_service_ = this->create_service<std_srvs::srv::Trigger>(
    "ur3/start_signal", 
    std::bind(&Main::startGameCallback, this, 
             std::placeholders::_1, std::placeholders::_2)
  );

  //create subscriber to receive movement message  
  ai_move_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/send_move", 10,
      std::bind(&Main::aiMoveCallback, this, std::placeholders::_1)
  );

  // Create publisher to indicate if the robot has completed the move
  move_complete_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "/move_complete", 10);
  
  RCLCPP_INFO(this->get_logger(), "Chess Robot Node initialized successfully");

}

void Main::run() {
  
  //make safety manager
  safety_manager_ = std::make_unique<SafetyManager>(shared_from_this());

  // Initialize robot (separate from constructor to handle MoveIt timing issues)
  robot_control_ = std::make_unique<RobotControl>(shared_from_this(), safety_manager_.get());
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
      //robot_control_->moveBoard();
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
  //two indexes
  int firstPos = result[0];
  int secondPos = result[1];
  //whether piece capture is necessary
  bool occupied = result[2];

  auto capturedPos = board_pos_->getCapturedPiecePosition();
  auto startPos = board_pos_->getBoardPosition(firstPos); 
  auto opponentPos = board_pos_->getBoardPosition(secondPos);

  // auto capturedJointValue = board_pos_->getJointValue(secondPos);
  // robot_control_->moveBoard(capturedJointValue);
  
  // for (int i = 63; i > -1; i--) {

  //   auto temp = board_pos_->getBoardPosition(i);
  //   //move to each consecutive grid piece, and record the joint values. in this way, we can find how to get to them immediately.
  //   //RCLCPP_INFO(this->get_logger(), "Moving to Index: %d", i);
  //   std::cout << "Moving to Index: " << i << std::endl;
  //   robot_control_->moveLinear(temp.x,temp.y,temp.z);


  // }
  
  // Execute the planned move sequence
  if (occupied) {

    //move opponent piece to black captured piece
    std::cout << "Moving captured white piece to white captured board" << std::endl;

    auto capturedJointValue = board_pos_->getJointValue(secondPos);
    robot_control_->moveBoard(capturedJointValue);
    
    //proceed with movement 
    //robot_control_->moveLinear(opponentPos.x,opponentPos.y,opponentPos.z);
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

auto jointValue = board_pos_->getJointValue(firstPos);
robot_control_->moveBoard(jointValue);

//move main piece 
//robot_control_->moveLinear(startPos.x,startPos.y,startPos.z);
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
