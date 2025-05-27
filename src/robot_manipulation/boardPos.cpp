#include "boardPos.h"
#include "robotControl.h"

// Constructor
BoardPos::BoardPos() {
    initialiseBoard();
    initialiseCapturedBoards();
}


void BoardPos::initialiseBoard() {
    for (int i = 0; i < 64; i++) {
        int file = i / 8; //files are columns
        int rank = i % 8; //ranks are rows 

        //X: Board spans -150 to +150mm, square centers at odd values
        board[i].position.x = (-0.15 + 0.01875) + 0.0375 * (file);
        //Y: Board spans 200 to 500mm, square centers at odd values  
        board[i].position.y = (0.13 + 0.01875) + 0.0375 * (rank); 
        board[i].position.z = 0.047;
    }
}

void BoardPos::initialiseCapturedBoards() {
    for (int i = 0; i < 16; i++) {

        int row = i / 8;
        int col = i % 8;
        

        //40mm offset between chessboard and array
        whiteCapturedPieces[i].position.x = (-0.150 - 0.040 - 0.075 + 0.01875) + 0.0375 * row;
        whiteCapturedPieces[i].position.y = (0.13 + 0.01875) + 0.0375 * col;
        whiteCapturedPieces[i].position.z = 0.047;
        whiteCapturedPieces[i].full = false;

        blackCapturedPieces[i].position.x = (0.150 + 0.040 + 0.01875) + 0.0375 * row;       
        blackCapturedPieces[i].position.y = (0.13 + 0.01875) + 0.0375 * col;
        blackCapturedPieces[i].position.z = 0.047;
        blackCapturedPieces[i].full = false;

    }
    
}

bool BoardPos::isPawnPromotion(int destinationIndex) {

    // Check if a pawn has a reached the opposite end of the board
    // check if destination is on the 8th rank (indices 56-63)

    return false;

}

std::vector<int> BoardPos::chessNotationToIndex(const std::string& notation) {
    if (notation.length() != 5) {
        std::cerr << "Invalid chess notation: " << notation << std::endl;
        return {};
    }
    
    char file = std::toupper(notation[0]); // Convert to uppercase (A-H)
    char rank = notation[1]; // Numeric rank (1-8)
    
    char file2 = std::toupper(notation[2]);
    char rank2 = notation[3];

    char value = notation[4];

    if (file < 'A' || file > 'H' || rank < '1' || rank > '8' || file2 < 'A' || file2 > 'H' || rank2 < '1' || rank2 > '8' || (value != '0' && value != '1') ) {
        std::cerr << "Invalid chess notation: " << notation << std::endl;
        return {};
    }
    
    // Calculate array index (A1 is 0, A2 is 1, ..., H8 is 63) - column-major order
    int col = file - 'A';
    int row = rank - '1';
    int col2 = file2 - 'A';
    int row2 = rank2 - '1';
    int taken = value - '0'; //methods converts char into int

    int firstIndex = col * 8 + row;
    int secondIndex = col2 * 8 + row2;

    return {firstIndex, secondIndex, taken};
}

BoardPos::Position3D BoardPos::getCapturedPiecePosition() {
    // Your existing logic for finding empty captured piece slot
    for (int i = 0; i < whiteCapturedPieces.size(); i++) {
        if (whiteCapturedPieces[i].full == false) {
            whiteCapturedPieces[i].full = true;
            return whiteCapturedPieces[i].position;
        }
    }
    throw std::runtime_error("No empty slot found in captured board!");
}

BoardPos::Position3D BoardPos::getBoardPosition(int index) {
    if (index < 0 || index >= 64) {
        throw std::out_of_range("Board index out of range");
    }
    return board[index].position;
}
