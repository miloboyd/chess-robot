#include "boardPos.h"

class BoardPos {
public:
    // Constructor
    BoardPos(bool isWhite) {
        initialiseBoard();
        placePieces(isWhite);
        initialiseCapturedBoards();
    }

private:
    enum class Color {
        WHITE,
        BLACK
    };

    enum class PieceType {
        PAWN,
        ROOK,
        KNIGHT,
        BISHOP,
        QUEEN,
        KING
    };

    struct Position3D {
        double x;
        double y;
        double z;
    };

    struct Piece {
        Color color;
        PieceType type;
    };

    enum class GridID {
        A1, A2, A3, A4, A5, A6, A7, A8,
        B1, B2, B3, B4, B5, B6, B7, B8,
        C1, C2, C3, C4, C5, C6, C7, C8,
        D1, D2, D3, D4, D5, D6, D7, D8,
        E1, E2, E3, E4, E5, E6, E7, E8,
        F1, F2, F3, F4, F5, F6, F7, F8,
        G1, G2, G3, G4, G5, G6, G7, G8,
        H1, H2, H3, H4, H5, H6, H7, H8
    };

    struct Square {
        Position3D position;
        std::optional<Piece> piece; // Using optional to represent empty squares
        GridID gridID;
    };

    struct CapturedSquare {
        Position3D position;
        std::optional<Piece> piece; // Using optional to represent empty positions
        GridID gridID;
    };

    //INITIALISE BOARD ARRAYS 
    std::array<Square, 64> board; // 8x8 board flattened to 1D array
    std::array<CapturedSquare, 16> whiteCapturedPieces;
    std::array<CapturedSquare, 16> blackCapturedPieces;

    void initialiseBoard() {
        for (int i = 0; i < 63; i++) {
            int row = i / 8;
            int col = i % 8;

            //18.75 constant to centre the pieces in the board grid
            board[i].position.x = (-150 + 18.75) + 37.5 * (row + 1); //150 constant to centre the board with robot positioning
            board[i].position.y = (50 + 18.75) + 37.5 * (col + 1); //50 constant to offer space between the robot and the board
            board[i].position.z = 10;
            board[i].gridID = static_cast<GridID>(i);
            board[i].piece = std::nullopt; // No piece on the square yet
        }
    }

    void initialiseCapturedBoards() {
        for (int i = 0; i < 16; i++) {

            int row = i / 8;
            int col = i % 8;

            whiteCapturedPieces[i].position.x = (-150 - 70 - 75 + 18.75) + 37.5 * (i + 1);
            whiteCapturedPieces[i].position.y = (50 + 18.75) + 37.5 * (i + 1);
            whiteCapturedPieces[i].position.z = 10;
            whiteCapturedPieces[i].gridID = static_cast<GridID>(i);
            whiteCapturedPieces[i].piece = std::nullopt; // No piece on the square yet

            blackCapturedPieces[i].position.x = (150 + 70 + 18.75) + 37.5 * (i + 1);       
            blackCapturedPieces[i].position.y = (50 + 18.75) + 37.5 * (i + 1);
            blackCapturedPieces[i].position.z = 10;
            blackCapturedPieces[i].gridID = static_cast<GridID>(i);
            blackCapturedPieces[i].piece = std::nullopt; // No piece on the square yet
        }
        
    }

    // Place pieces according to initial chess setup
    void placePieces(bool isWhite) {

        // Define piece layouts for both colors
        const PieceType backRowWhite[8] = {
            PieceType::ROOK, PieceType::KNIGHT, PieceType::BISHOP, PieceType::KING,
            PieceType::QUEEN, PieceType::BISHOP, PieceType::KNIGHT, PieceType::ROOK
        };
        
        const PieceType backRowBlack[8] = {
            PieceType::ROOK, PieceType::KNIGHT, PieceType::BISHOP, PieceType::QUEEN,
            PieceType::KING, PieceType::BISHOP, PieceType::KNIGHT, PieceType::ROOK
        };

        if (isWhite) {
            // Setup pieces for white orientation
            
            // White pieces (bottom rows)
            for (int col = 0; col < 8; col++) {

                // Back row (index 56-63)
                board[56 + col].piece = Piece{Color::WHITE, backRowWhite[col]};
                // Pawns (row 48-55)
                board[48 + col].piece = Piece{Color::WHITE, PieceType::PAWN};

                //Black Pieces (top rows)
                // Back row (index 0-7)
                board[col].piece = Piece{Color::BLACK, backRowWhite[col]};
                // Pawns (row 8-15)
                board[8 + col].piece = Piece{Color::BLACK, PieceType::PAWN};
            }
        } else {

            // White pieces (top rows)
            for (int col = 0; col < 8; col++) {

                // Back row (index 0-7)
                board[col].piece = Piece{Color::WHITE, backRowBlack[col]};
                // Pawns (row 8-15)
                board[8 + col].piece = Piece{Color::WHITE, PieceType::PAWN};

                //Black Pieces (bottom rows)
                // Back row (index 56-63)
                board[56 + col].piece = Piece{Color::BLACK, backRowBlack[col]};
                // Pawns (row 48-55)
                board[48 + col].piece = Piece{Color::BLACK, PieceType::PAWN};
            }
        }
        // Middle rows (2-5) remain empty (nullopt)
    }
    
public:
    
    bool movePiece(const int source, const int destination) {  
        
        if (source < 0 || source >= 64 || destination < 0 || destination >= 64) {
            throw std::out_of_range("Source or destination index out of range");
        }

        // Check if the source square has a piece
        if (!board[source].piece) {
            throw std::invalid_argument("No piece at source square");
        }

        //Get the piece information
    }

    std::pair<int, int> notationToIndex(const std::string notation) {  

        if (notation.length() != 4) {
            throw std::invalid_argument("Input must be 4 characters long");
        }

        char sourceFile = toupper(notation[0]);
        char sourceRank = notation[1];
        char destinationFile = toupper(notation[2]);
        char destinationRank = notation[3];

        // Convert file (A-H) to index (0-7)
        int sourceCol = sourceFile - 'A';
        int sourceRow = '8' - sourceRank;
        int sourceIndex = sourceRow * 8 + sourceCol;

        // Convert destination file (A-H) to index (0-7)
        int destinationCol = destinationFile - 'A';
        int destinationRow = '8' - destinationRank;
        int destinationIndex = destinationRow * 8 + destinationCol;

        //test print
        //std::cout << "First: " << first << ", Second: " << second << std::endl;

        return {sourceIndex, destinationIndex};
    }

    bool isValidMove(int sourceIndex, int destinationIndex, const Piece& piece) {

    }

    /** 
    visualiseBoard {

    [ ][ ][ ][ ][ ][ ][ ][ ]
    [ ][ ][ ][ ][ ][ ][ ][ ]
    [ ][ ][ ][ ][ ][ ][ ][ ]
    [ ][ ][ ][ ][ ][ ][ ][ ]
    [ ][ ][ ][ ][ ][ ][ ][ ]
    [ ][ ][ ][ ][ ][ ][ ][ ]
    [ ][ ][ ][ ][ ][ ][ ][ ]
    [ ][ ][ ][ ][ ][ ][ ][ ]
        

    }
    */
};