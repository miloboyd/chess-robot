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

        robotControl robot;

        // Handle capturing opponent's piece
        if (isTaken == true) {

            //move opponent piece to black captured piece
            std::cout << "Moving captured white piece to white captured board" << std::endl;
             
            for (int i = 0; i < whiteCapturedPieces.size(); i++) {
                if (whiteCapturedPieces[i].full == false) {
                    whiteCapturedPieces[i].full = true;
                    capturedIndex = i;
                    std::cout << "Piece placed in captured board at position " << i << std::endl;
                }
            }
            // If we reach here, there's no empty slot (shouldn't happen in a standard chess game)
            std::cout << "Warning: No empty slot found in captured board!" << std::endl;
            return false;

            robot.moveRobot(board[finishIndex].position.x, board[finishIndex].position.y, board[finishIndex].position.z + 2);
            robot.pickUpPiece();
            robot.moveRobot(whiteCapturedPieces[capturedIndex].position.x, whiteCapturedPieces[capturedIndex].position.y, whiteCapturedPieces[capturedIndex].position.z + 2);
            robot.placePiece();
        }

        //Check for pawn promotion
        if (isPawnPromotion(destinationIndex)) {
            // handlePawnPromotion(destination);
            std::cout << "Pawn promotion detected! (Implementation pending)" << std::endl;
            //when pulling piece, remove taken piece type 
        }

        robot.moveRobot(board[startIndex].position.x, board[startIndex].position.y, board[startIndex].position.z + 2);
        robot.pickUpPiece();
        robot.moveRobot(whiteCapturedPieces[finishIndex].position.x, whiteCapturedPieces[finishIndex].position.y, whiteCapturedPieces[finishIndex].position.z + 2);
        robot.placePiece();

        return true;

    }   

    bool BoardPos::isPawnPromotion(int destinationIndex) {
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

