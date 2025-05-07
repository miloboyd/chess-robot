#include "boardPos.h"

    // Constructor
    BoardPos::BoardPos() {
        initialiseBoard();
        placePieces();
        initialiseCapturedBoards();
    }


    void BoardPos::initialiseBoard() {
        for (int i = 0; i < 64; i++) {
            int row = i / 8;
            int col = i % 8;

            //18.75 constant to centre the pieces in the board grid
            board[i].position.x = (-150 + 18.75) + 37.5 * (row + 1); //150 constant to centre the board with robot positioning
            board[i].position.y = (50 + 18.75) + 37.5 * (col + 1); //50 constant to offer space between the robot and the board
            board[i].position.z = 10;
        }
    }

    void BoardPos::initialiseCapturedBoards() {
        for (int i = 0; i < 16; i++) {

            int row = i / 8;
            int col = i % 8;

            whiteCapturedPieces[i].position.x = (-150 - 70 - 75 + 18.75) + 37.5 * (i + 1);
            whiteCapturedPieces[i].position.y = (50 + 18.75) + 37.5 * (i + 1);
            whiteCapturedPieces[i].position.z = 10;
            whiteCapturedPiece[i].full = false;

            blackCapturedPieces[i].position.x = (150 + 70 + 18.75) + 37.5 * (i + 1);       
            blackCapturedPieces[i].position.y = (50 + 18.75) + 37.5 * (i + 1);
            blackCapturedPieces[i].position.z = 10;
            blackCapturedPieces[i].full = false;

        }
        
    }
    
    bool BoardPos::movePiece(const std::string& start, const std::string& finish, bool isTaken) {  

        int startIndex = chessNotationToIndex(start);
        int finishIndex = chessNotationToIndex(finish);
        
        // Handle capturing opponent's piece
        if (isTaken == true) {
            
            //move opponent piece to black captured piece
            std::cout << "Moving captured white piece to white captured board" << std::endl;
            
            robotControl robot;

            robot.moveRobot(board[startIndex].position.x, board[startIndex].position.y, board[startIndex].position.z + 2);
            robot.pickUpPiece();
            robot.moveRobot()
            
            //must keep record of whiteCapturedPieces grid
            
            
            //when placing piece, add taken boolean 
            //iterate through whiteCapturedPieces to check for value
                //place piece, update grid to give taken value
        }

        // Check for pawn promotion
        /**        if (isPawnPromotion(source, destination, sourcePiece)) {
            // handlePawnPromotion(destination);
            std::cout << "Pawn promotion detected! (Implementation pending)" << std::endl;

            //when pulling piece, remove taken piece type 
        }
        */


        //move piece from start to finish coord
        //std::cout << "Moving piece from" << grid.ID << "to" << grid.ID << std::endl;
        //move robot()

        return true;

    }    

    bool BoardPos::isPawnPromotion(int sourceIndex, int destinationIndex, const Piece& piece) {

        //move captured 
        return false;

    }


    bool BoardPos::moveToCapturedBoard(const Piece& piece) {
        
        // Find the first empty slot in the captured board
        for (int i = 0; i < whiteCapturedPieces.size(); i++) {
            if (!whiteCapturedPieces[i].piece.has_value()) {
                // Place the captured piece in this slot
                whiteCapturedPieces[i].piece = piece;
                std::cout << "Piece placed in captured board at position " << i << std::endl;
                return true;
            }
        }
        
        // If we reach here, there's no empty slot (shouldn't happen in a standard chess game)
        std::cout << "Warning: No empty slot found in captured board!" << std::endl;
        return false;
    }

    int chessNotationToIndex(const std::string& notation) {
        if (notation.length() != 2) {
            std::cerr << "Invalid chess notation: " << notation << std::endl;
            return -1;
        }
        
        char file = std::toupper(notation[0]); // Convert to uppercase (A-H)
        char rank = notation[1]; // Numeric rank (1-8)
        
        if (file < 'A' || file > 'H' || rank < '1' || rank > '8') {
            std::cerr << "Invalid chess notation: " << notation << std::endl;
            return -1;
        }
        
        // Calculate array index (A1 is 0, A2 is 1, ..., H8 is 63) - column-major order
        int col = file - 'A';
        int row = rank - '1';
        
        return col * 8 + row;
    }

