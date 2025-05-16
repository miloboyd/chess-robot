#include <rclcpp/rclcpp.hpp>
#include "boardPos.h"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    std::cout << "=== Simple BoardPos Test Program ===" << std::endl;
    
    // Create board position object
    std::cout << "Initializing BoardPos object..." << std::endl;
    BoardPos chessBoard;
    std::cout << "BoardPos initialized successfully!" << std::endl;
    
    // Test parameters
    std::string startPosition = "B5";
    std::string endPosition = "C5";
    bool isCaptured = false;
    
    // Run the test
    std::cout << "\nTesting movePiece from " << startPosition << " to " << endPosition 
              << " (Capture: " << (isCaptured ? "Yes" : "No") << ")" << std::endl;
    
    bool moveResult = chessBoard.movePiece(startPosition, endPosition, isCaptured);
    
    std::cout << "Move result: " << (moveResult ? "Success" : "Failure") << std::endl;
    
    // Try another test with capture
    startPosition = "C5";
    endPosition = "D6";
    isCaptured = true;
    
    std::cout << "\nTesting movePiece from " << startPosition << " to " << endPosition 
              << " (Capture: " << (isCaptured ? "Yes" : "No") << ")" << std::endl;
    
    moveResult = chessBoard.movePiece(startPosition, endPosition, isCaptured);
    
    std::cout << "Move result: " << (moveResult ? "Success" : "Failure") << std::endl;
    
    std::cout << "\nTest program completed!" << std::endl;
    
    rclcpp::shutdown();
    return 0;
}