/**
 * @file boardPos.h
 * @brief Header file for boardPos implementation
 * @author MiloBoyd
 * @date 2025-05-07
 */

#ifndef BOARD_POS_H
#define BOARD_POS_H

#include <array>
#include <optional>
#include <string>
#include <iostream>
#include <cctype>
#include <utility>

/**
 * @brief A class that allocates physical positions to chess piece grids. 
 * 
 * This class defines the position of the board with respect to the robot workspace, as well as performing movement of the robot arm. Each grid has a physical position defined to it,
 * and when the corresponding chess pieces are passed through from the ___ Class, the robot arm movement will be performed. 
 * 
 * @note will need to install packages and dependencies for usage. These include moveIt for ROS2 humble, Universal_Robots_ROS2_Driver.
 * @see AIClass, robotControl
 */
class BoardPos {

public:

    /**
     * @brief Contructor that initialises the chessboard physical properties. 
     * Initialises the chessboard and captured chessboard positions with respect to the robot arm base.  
     */
    BoardPos();

    /**
     * @brief Calls for robot arm movement to manipulate chess pieces. 
     * 
     * @param start, finish Grid positions in chess notation (e.g. A4, C6). 
     * @param isTaken Value to identify whether capturing a piece is necessary. 
     * 
     * @returns Boolean value on whether the operation was successful. 
     * @see robotControl, chessNotationToIndex, isPawnPromotion, placePieces
     */
    bool movePiece(const std::string& start, const std::string& fnish, bool isTaken);
private:

    struct Position3D {
        double x;
        double y;
        double z;
    };

    struct Square {
        Position3D position;
    };

    struct CapturedSquare {
        Position3D position;
        bool full;
    };

    std::array<Square, 64> board; 
    std::array<CapturedSquare, 16> whiteCapturedPieces;
    std::array<CapturedSquare, 16> blackCapturedPieces;

    void initialiseBoard();
    void initialiseCapturedBoards();
    
    bool isPawnPromotion(int sourceIndex, int destinationIndex, const Piece& piece);
    bool moveToCapturedBoard(const Piece& piece);
    int chessNotationToIndex(const std::string& notation);

};

#endif // BOARD_POS_H