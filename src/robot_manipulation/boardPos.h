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
     * @see robotControl, chessNotationToIndex, isPawnPromotion, placePieces.
     */
    bool movePiece(const std::string& start, const std::string& fnish, bool isTaken);

private:


    struct Position3D {
        double x; ///< X-coordinate in the robot's workspace (in meters)
        double y; ///< Y-coordinate in the robot's workspace (in meters)
        double z; ///< Z-coordinate or height in the robot's workspace (in meters)
    };

    struct Square {
        Position3D position; ///< 3D position of this chess square in the robot's workspace
    };

    struct CapturedSquare {
        Position3D position; ///< 3D position where captured pieces should be placed 
        bool full; ///< Indicates whether this position already contains a captured piece
    };

    /** Chess board represented as a 1D array of 64 squares (8x8 flattened grid) */
    std::array<Square, 64> board; 

    /** Storage locations for captured white pieces (maximum 16 pieces) */
    std::array<CapturedSquare, 16> whiteCapturedPieces;

    /** Storage locations for captured black pieces (maximum 16 pieces) */
    std::array<CapturedSquare, 16> blackCapturedPieces;



    /**
     * @brief Initialise chess array and physical positions. 
     * 
     * This method initialises an array of length 64, flattened from an 8x8 grid.
     */
    void initialiseBoard();

    /**
     * @brief Initialise captured board array and physical positions. 
     * 
     * This method initialises two board arrays that captured pieces will be placed in. They will stand 
     * adjacent to the chessboard and each board contains either white or black pieces. 
     */
    void initialiseCapturedBoards();
    
    /**
     * @brief Checks for pawn promotion instance.
     * 
     * This method performs operations to assess whether a pawn can be promoted at the opposite end of the grid to a higher order piece. The robot can choose 
     * what piece the pawn will be replaced with. 
     * 
     * @param destinationIndex Index will determine the grid position and eligibility.
     * @returns Eligbility of pawn promotion case.
     */
    bool isPawnPromotion(int destinationIndex);

    /**
     * @brief Transforms chess notation to board index.
     * 
     * This method takes chess notation as an input and transforms the grid position into an index that corresponds to an 8x8 array 
     * flattened into a 1x64 array.
     * 
     * @param notation A string input in chess notation (e.g. A4).
     * @returns Int of the board index that corresponds to the chess notation input.
     */
    int chessNotationToIndex(const std::string& notation);

};

#endif // BOARD_POS_H