#include "boardPos.h"

class BoardPos {
public:
    // Constructor
    BoardPos() {
        initialiseBoard();
        placePieces();
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
        for (int i = 0; i < 64; i++) {
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
    void placePieces() {

        // Define piece layouts for both colors
        const PieceType backRowWhite[8] = {
            PieceType::ROOK, PieceType::KNIGHT, PieceType::BISHOP, PieceType::KING,
            PieceType::QUEEN, PieceType::BISHOP, PieceType::KNIGHT, PieceType::ROOK
        };

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
    // Middle rows (2-5) remain empty (nullopt)
    }
    
public:
    
    bool movePiece(const int source, const int destination) {  
        
        if (source < 0 || source >= 64 || destination < 0 || destination >= 64) {
            std::cout << "Error: Source or destination index out of range" << std::endl;
            throw std::out_of_range("Source or destination index out of range");
        }

        // Check if the source square has a piece
        if (!board[source].piece) {
            std::cout << "Error: No piece at source square" << std::endl;
            throw std::invalid_argument("No piece at source square");
        }

        //Get the piece information
        Piece sourcePiece = board[source].piece.value();

        //isvalid move function make sure move is possible with the piece type
        if (!isValidMove(source, destination, sourcePiece)) {
            std::cout << "Error: Invalid move for the piece type" << std::endl;
            throw std::invalid_argument("Invalid move for the piece type");
        }
        

        //check if the destination square is occupied by an allied piece
        if (board[destination].piece.has_value() && 
            board[destination].piece->color == sourcePiece.color) {
            std::cout << "Error: Cannot capture your own piece" << std::endl;
            throw std::invalid_argument("Cannot capture your own piece");
        }


        // Handle capturing opponent's piece
        if (board[destination].piece.has_value()) {
            Piece capturedPiece = board[destination].piece.value();
            std::cout << "Capturing opponent's piece: " 
                    << getPieceTypeString(capturedPiece.type) << std::endl;
            
            // Determine which captured board to use based on the captured piece's color
            if (capturedPiece.color == Color::WHITE) {
                // Move to white captured board
                moveToCapturedBoard(capturedPiece, true);
                // movementFunction(destination, whiteCapturedLocation); // Robot arm movement
                std::cout << "Moving captured white piece to white captured board" << std::endl;
            } else {
                // Move to black captured board
                moveToCapturedBoard(capturedPiece, false);
                // movementFunction(destination, blackCapturedLocation); // Robot arm movement
                std::cout << "Moving captured black piece to black captured board" << std::endl;
            }
        }

        board[destination].piece = sourcePiece;
        board[source].piece = std::nullopt;

        // Move the piece on the board state
        board[destination].piece = sourcePiece;
        board[source].piece = std::nullopt;
        
        // Call robot arm movement function to execute the move
        // movementFunction(source, destination); // Robot arm movement
        std::cout << "Moving piece from " << getSquareNotation(source) 
                << " to " << getSquareNotation(destination) << std::endl;

        // Check for pawn promotion
        if (isPawnPromotion(source, destination, sourcePiece)) {
            // handlePawnPromotion(destination);
            std::cout << "Pawn promotion detected! (Implementation pending)" << std::endl;
        }

        return true;

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
        
        //turn 1D indices into 2D coordinates
        int sourceRow = sourceIndex / 8;
        int sourceCol = sourceIndex % 8;
        int destinationRow = destinationIndex / 8;
        int destinationCol = destinationIndex % 8;

        //calculate the differences between positions
        int rowDiff = destinationRow - sourceRow;
        int colDiff = destinationCol - sourceCol;

        //Create a switch case based on the piece type
        switch (piece.type) {
            case PieceType::PAWN: {
                std::cout << "Validating pawn move..." << std::endl;
                
                //direction depends on piece colour
                int forwardDirection = (piece.color == Color::WHITE) ? -1 : 1;
    
                //regular move: 1 square forward (no capture)
                if (colDiff == 0 && rowDiff == forwardDirection) {
                    // Check if destination is empty
                    if (!board[destinationIndex].piece.has_value()) {
                        std::cout << "Valid pawn move: single square forward" << std::endl;
                        return true;
                    } else {
                        std::cout << "Failed pawn move: destination square is occupied" << std::endl;
                        return false;
                    }
                }
                
                //first move: 2 squares forward
                else if (colDiff == 0 && rowDiff == 2 * forwardDirection) {
                    // Check if this is the pawn's starting position
                    bool isStartingPosition = (piece.color == Color::WHITE && sourceRow == 6) || 
                                            (piece.color == Color::BLACK && sourceRow == 1);
                    
                    if (!isStartingPosition) {
                        std::cout << "Failed pawn move: two squares forward only allowed from starting position" << std::endl;
                        return false;
                    }
                    
                    // Check if both destination and the square in between are empty
                    int middleIndex = sourceIndex + (8 * forwardDirection);
                    if (!board[middleIndex].piece.has_value() && !board[destinationIndex].piece.has_value()) {
                        std::cout << "Valid pawn move: two squares forward from starting position" << std::endl;
                        return true;
                    } else {
                        std::cout << "Failed pawn move: path blocked for two-square move" << std::endl;
                        return false;
                    }
                }
                
                //capture move: 1 square diagonally 
                else if (abs(colDiff) == 1 && rowDiff == forwardDirection) {
                    // Check if destination has an opponent's piece
                    if (board[destinationIndex].piece.has_value()) {
                        if (board[destinationIndex].piece->color != piece.color) {
                            std::cout << "Valid pawn move: diagonal capture" << std::endl;
                            return true;
                        } else {
                            std::cout << "Failed pawn move: can't capture your own piece" << std::endl;
                            return false;
                        }
                    } else {
                        std::cout << "Failed pawn move: diagonal move requires capture" << std::endl;
                        return false;
                    }
                }
                
                std::cout << "Failed pawn move: invalid move pattern" << std::endl;
                return false;
            }

            case PieceType::ROOK: {
                std::cout << "Validating rook move..." << std::endl;
                if (sourceRow != destinationRow && sourceCol != destinationCol) {
                    std::cout << "Failed rook move: must move along rank or file" << std::endl;
                    return false;

                bool pathClear = isStraightPathClear(sourceIndex, destinationIndex);
                if (pathClear) {
                    std::cout << "Valid rook move" << std::endl;
                } else {
                    std::cout << "Failed rook move: path blocked" << std::endl;
                }
                return pathClear;
                }
            }

            case PieceType::BISHOP: {
                std::cout << "Validating bishop move..." << std::endl;
                if (abs(rowDiff) != abs(colDiff)) {
                    std::cout << "Failed bishop move: must move diagonally" << std::endl;
                    return false;
                
                bool pathClear = isDiagonalPathClear(sourceIndex, destinationIndex);
                if (pathClear) {
                    std::cout << "Valid bishop move" << std::endl;
                } else {
                    std::cout << "Failed bishop move: path blocked" << std::endl;
                }
                return pathClear;
                }
            }
                
            case PieceType::QUEEN: {
                std::cout << "Validating queen move..." << std::endl;
                bool pathClear = false;
                
                if (sourceRow == destinationRow || sourceCol == destinationCol) {
                    pathClear = isStraightPathClear(sourceIndex, destinationIndex);
                    if (pathClear) {
                        std::cout << "Valid queen move: straight" << std::endl;
                    } else {
                        std::cout << "Failed queen move: straight path blocked" << std::endl;
                    }
                }
                else if (abs(rowDiff) == abs(colDiff)) {
                    pathClear = isDiagonalPathClear(sourceIndex, destinationIndex);
                    if (pathClear) {
                        std::cout << "Valid queen move: diagonal" << std::endl;
                    } else {
                        std::cout << "Failed queen move: diagonal path blocked" << std::endl;
                    }
                } else {
                    std::cout << "Failed queen move: must move straight or diagonally" << std::endl;
                }
                return pathClear;
            }

            case PieceType::KING: {
                std::cout << "Validating king move..." << std::endl;
                //movement restricted to one change in either row or column diff
                if (abs(rowDiff) <= 1 && abs(colDiff) <= 1) {
                    std::cout << "Valid king move" << std::endl;
                    return true;
                } else {
                    std::cout << "Failed king move: can only move one square in any direction" << std::endl;
                    return false;
                }
            }

            case PieceType::KNIGHT: {
                std::cout << "Validating knight move..." << std::endl;
                if ((abs(rowDiff) == 2 && abs(colDiff) == 1) || (abs(rowDiff) == 1 && abs(colDiff) == 2)) {
                    std::cout << "Valid knight move" << std::endl;
                    return true;
                } else {
                    std::cout << "Failed knight move: must move in L-shape (2+1)" << std::endl;
                    return false;
                }
            }

            default:
            std::cout << "Invalid piece type" << std::endl;
            return false;
            
        }
        }
    }   

    bool isPawnPromotion(int sourceIndex, int destinationIndex, const Piece& piece) {
        return false;
    }

    bool isStraightPathClear(int sourceIndex, int destinationIndex) {
        std::cout << "Checking straight path clearance..." << std::endl;
        
        int sourceRow = sourceIndex / 8;
        int sourceCol = sourceIndex % 8;
        int destRow = destinationIndex / 8;
        int destCol = destinationIndex % 8;
        
        // Determine direction of movement
        int rowStep = 0;
        int colStep = 0;
        
        if (sourceRow < destRow) rowStep = 1;       // Moving down
        else if (sourceRow > destRow) rowStep = -1; // Moving up
        
        if (sourceCol < destCol) colStep = 1;       // Moving right
        else if (sourceCol > destCol) colStep = -1; // Moving left
        
        // Starting from the square after the source, check each square until just before the destination
        int currentRow = sourceRow + rowStep;
        int currentCol = sourceCol + colStep;
        
        while ((currentRow != destRow) || (currentCol != destCol)) {
            int currentIndex = currentRow * 8 + currentCol;
            
            // If there's a piece in the path, the path is blocked
            if (board[currentIndex].piece.has_value()) {
                std::cout << "Path blocked at row " << currentRow << ", col " << currentCol << std::endl;
                return false;
            }
            
            // Move to the next square in the path
            currentRow += rowStep;
            currentCol += colStep;
        }
        
        
        std::cout << "Straight path is clear" << std::endl;
        return true;
    }

    bool isDiagonalPathClear(int sourceIndex, int destinationIndex) {
        std::cout << "Checking diagonal path clearance..." << std::endl;
        
        int sourceRow = sourceIndex / 8;
        int sourceCol = sourceIndex % 8;
        int destRow = destinationIndex / 8;
        int destCol = destinationIndex % 8;
        
        // Verify the move is actually diagonal (abs(rowDiff) == abs(colDiff))
        if (abs(sourceRow - destRow) != abs(sourceCol - destCol)) {
            std::cout << "Not a diagonal move" << std::endl;
            return false;
        }
        
        // Determine direction of diagonal movement
        int rowStep = (destRow > sourceRow) ? 1 : -1;
        int colStep = (destCol > sourceCol) ? 1 : -1;
        
        // Starting from the square after the source, check each square until just before the destination
        int currentRow = sourceRow + rowStep;
        int currentCol = sourceCol + colStep;
        
        while (currentRow != destRow && currentCol != destCol) {
            int currentIndex = currentRow * 8 + currentCol;
            
            // If there's a piece in the path, the path is blocked
            if (board[currentIndex].piece.has_value()) {
                std::cout << "Path blocked at row " << currentRow << ", col " << currentCol << std::endl;
                return false;
            }
            
            // Move to the next square in the path
            currentRow += rowStep;
            currentCol += colStep;
        }
        
        // Check if destination has a piece of the same color (can't capture your own piece)
        if (board[destinationIndex].piece.has_value() && 
            board[sourceIndex].piece.has_value() && 
            board[destinationIndex].piece->color == board[sourceIndex].piece->color) {
            std::cout << "Destination contains a piece of the same color" << std::endl;
            return false;
        }
        
        std::cout << "Diagonal path is clear" << std::endl;
        return true;
    }

    // Helper function to convert piece type to string for debug output
    std::string getPieceTypeString(PieceType type) {
        switch (type) {
            case PieceType::PAWN: return "Pawn";
            case PieceType::ROOK: return "Rook";
            case PieceType::KNIGHT: return "Knight";
            case PieceType::BISHOP: return "Bishop";
            case PieceType::QUEEN: return "Queen";
            case PieceType::KING: return "King";
            default: return "Unknown";
        }
    }
    
    // Helper function to convert board index to algebraic notation (e.g., A1, C4)
    std::string getSquareNotation(int index) {
        int row = index / 8;
        int col = index % 8;
        char file = 'A' + col;
        char rank = '8' - row;
        return std::string(1, file) + std::string(1, rank);
    }

    bool moveToCapturedBoard(const Piece& piece, bool isWhitePiece) {
        // Determine which captured board to use
        auto& capturedBoard = isWhitePiece ? whiteCapturedPieces : blackCapturedPieces;
        
        // Find the first empty slot in the captured board
        for (int i = 0; i < capturedBoard.size(); i++) {
            if (!capturedBoard[i].piece.has_value()) {
                // Place the captured piece in this slot
                capturedBoard[i].piece = piece;
                std::cout << "Piece placed in captured board at position " << i << std::endl;
                return true;
            }
        }
        
        // If we reach here, there's no empty slot (shouldn't happen in a standard chess game)
        std::cout << "Warning: No empty slot found in captured board!" << std::endl;
        return false;
    }
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