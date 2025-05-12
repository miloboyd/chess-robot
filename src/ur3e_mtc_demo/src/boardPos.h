#ifndef BOARD_POS_H
#define BOARD_POS_H

#include <array>
#include <optional>
#include <string>



/**
 * @class BoardPos
 * @brief Board Position Class
 * 
 * This class defines the dimensions and position of the board in the robot's workspace.
 * 
 */

class BoardPos {

public:

    //Constructor
    BoardPos(bool isWhite);



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

    void placePieces(bool isWhite);
    void initialiseBoard();
    std::optional<Piece> getPieceAt(int row, int col) const;
    bool movePiece(int fromRow, int fromCol, int toRow, int toCol);

    



};

#endif