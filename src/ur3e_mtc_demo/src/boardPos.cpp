#include "boardPos.h"

class BoardPos {
public:
    // Constructor
    BoardPos(bool isWhite) {
        initialiseBoard();
        placePieces(isWhite);
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

    struct Position3D {
        double x;
        double y;
        double z;
    };

    struct Piece {
        Color color;
        PieceType type;
    };

    struct Square {
        Position3D position;
        std::optional<Piece> piece; // Using optional to represent empty squares
        GridID gridID;
    };

    std::array<std::array<Square, 8>, 8> board;

    // Initialize the board with positions and grid IDs but no pieces
    void initialiseBoard() {
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                board[i][j].position.x = 18.75 * (i + 1);
                board[i][j].position.y = 18.75 * (j + 1);
                board[i][j].position.z = 10;
                // Convert linear index to GridID enum value
                board[i][j].gridID = static_cast<GridID>(i * 8 + j);
                // No piece on the square yet
                board[i][j].piece = std::nullopt;
            }
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
                // Back row (row 7)
                board[7][col].piece = Piece{Color::WHITE, backRowWhite[col]};
                
                // Pawns (row 6)
                board[6][col].piece = Piece{Color::WHITE, PieceType::PAWN};
            }
            
            // Black pieces (top rows)
            for (int col = 0; col < 8; col++) {
                // Back row (row 0)
                board[0][col].piece = Piece{Color::BLACK, backRowBlack[col]};
                
                // Pawns (row 1)
                board[1][col].piece = Piece{Color::BLACK, PieceType::PAWN};
            }
        } else {
            // Setup pieces for black orientation
            
            // Black pieces (bottom rows)
            for (int col = 0; col < 8; col++) {
                // Back row (row 7)
                board[7][col].piece = Piece{Color::BLACK, backRowBlack[col]};
                
                // Pawns (row 6)
                board[6][col].piece = Piece{Color::BLACK, PieceType::PAWN};
            }
            
            // White pieces (top rows)
            for (int col = 0; col < 8; col++) {
                // Back row (row 0)
                board[0][col].piece = Piece{Color::WHITE, backRowWhite[col]};
                
                // Pawns (row 1)
                board[1][col].piece = Piece{Color::WHITE, PieceType::PAWN};
            }
        }
        
        // Middle rows (2-5) remain empty (nullopt)
    }
    
public:
    // Add public methods as needed
    
    // Example: Get the piece at a specific position
    std::optional<Piece> getPieceAt(int row, int col) const {
        if (row >= 0 && row < 8 && col >= 0 && col < 8) {
            return board[row][col].piece;
        }
        return std::nullopt;
    }
    
    // Example: Move a piece
    bool movePiece(int fromRow, int fromCol, int toRow, int toCol) {
        if (fromRow < 0 || fromRow >= 8 || fromCol < 0 || fromCol >= 8 ||
            toRow < 0 || toRow >= 8 || toCol < 0 || toCol >= 8) {
            return false;
        }
        
        if (!board[fromRow][fromCol].piece) {
            return false; // No piece at source
        }
        
        // Move the piece
        board[toRow][toCol].piece = board[fromRow][fromCol].piece;
        board[fromRow][fromCol].piece = std::nullopt;
        
        return true;
    }
};