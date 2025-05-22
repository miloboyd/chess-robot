#include <rclcpp/rclcpp.hpp>
#include "boardPos.h"
#include "robotControl.h"
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    std::cout << "=== Simple BoardPos Test Program ===" << std::endl;

    try {

        std::cout << "Creating robot control..." << std::endl;
        auto robot_control = RobotControl::create();

        //create boardPos with robot dependency
        std::cout << "Initialising boardPos object..." << std::endl;
        BoardPos chessBoard(robot_control);
        std::cout << "BoardPos initialised successfully!" << std::endl;

        //setup executor
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(robot_control);
        auto spinner = std::thread([&executor]() { executor.spin(); });


        std::string move1 = "b5c51";
        std::string move2 = "c5d60";

        robot_control->moveBoard();

        //Test the moves 
        std::cout << "\nTesting movePiece: " << move1 << std::endl;
        bool moveResult = chessBoard.movePiece(move1);
        std::cout << "Move result: " << (moveResult ? "Success" : "Failure") << std::endl;
        
        std::cout << "\nTesting movePiece: " << move2 << std::endl;
        moveResult = chessBoard.movePiece(move2);
        std::cout << "Move result: " << (moveResult ? "Success" : "Failure") << std::endl;
        
        std::cout << "\nTest program completed!" << std::endl;
        
        rclcpp::shutdown();
        spinner.join();
    } 
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return -1;
    }
    
    
    return 0;
}