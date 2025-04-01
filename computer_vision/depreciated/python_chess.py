import numpy as np
import chess
import chess.pgn
from typing import Tuple, Optional, List

class ChessBoardAnalyzer:
    def __init__(self):
        # Initialize a chess board
        self.board = chess.Board()
        # Initialize PGN game
        self.game = chess.pgn.Game()
        self.game.headers["Event"] = "Computer Vision Chess Analysis"
        self.game.headers["Site"] = "Local Analysis"
        self.game.headers["Date"] = "????.??.??"
        self.game.headers["Round"] = "1"
        self.game.headers["White"] = "Player 1"
        self.game.headers["Black"] = "Player 2"
        self.game.headers["Result"] = "*"
        self.current_node = self.game
        
        # Keep track of the previous board state as a NumPy array
        self.previous_board_array = self.board_to_array(self.board)
    
    def board_to_array(self, board):
        """Convert a python-chess board to an 8x8 NumPy array"""
        board_array = np.zeros((8, 8), dtype=int)
        
        for square in chess.SQUARES:
            piece = board.piece_at(square)
            if piece:
                # Get rank and file (0-7)
                rank = chess.square_rank(square)
                file = chess.square_file(square)
                
                # Set value based on piece type and color
                # We use positive for white pieces, negative for black
                value = piece.piece_type
                if piece.color == chess.BLACK:
                    value = -value
                
                board_array[7-rank][file] = value
        print(board_array)
        return board_array
    
    def detect_move(self, new_board_array):
        """Detect the move by comparing the previous and new board states"""
        # Find changed squares
        changed_squares = np.where(self.previous_board_array != new_board_array)
        changed_rows, changed_cols = changed_squares
        
        if len(changed_rows) < 2:
            # No move detected or invalid
            return None
        
        # Find candidate moves
        candidate_moves = []
        
        # Get all legal moves from current board position
        for move in self.board.legal_moves:
            # Make the move on a temporary board
            temp_board = chess.Board(self.board.fen())
            temp_board.push(move)
            
            # Convert to array and compare with new_board_array
            temp_array = self.board_to_array(temp_board)
            
            # Calculate number of matching positions
            matches = np.sum(temp_array == new_board_array)
            total_squares = 64
            
            # Add to candidates with match score
            if matches >= (total_squares - 4):  # Allow for some discrepancy
                candidate_moves.append((move, matches))
                
                # Debug information
                if matches == total_squares:
                    print(f"Perfect match found: {move}")
        
        # If no candidates found, return None
        if not candidate_moves:
            print("No candidate moves found that match the new board state")
            # Print current board FEN for debugging
            print(f"Current board: {self.board.fen()}")
            return None
        
        # Get the best match
        best_match = max(candidate_moves, key=lambda x: x[1])
        best_move, match_score = best_match
        
        print(f"Best move match: {best_move} with {match_score}/{64} matching squares")
        
        return best_move
    
    def update_board(self, new_board_array):
        """Update the board with a newly detected position and return the PGN move"""
        # Detect move
        move = self.detect_move(new_board_array)
        
        if move is None:
            # No valid move detected
            print("No valid move detected")
            return None
        
        try:
            # Check if move is legal in current position
            if move not in self.board.legal_moves:
                print(f"Move {move} is not legal in current position")
                return None
            
            # Get SAN notation of the move before pushing it
            san_move = self.board.san(move)
            
            # Make the move on the board
            self.board.push(move)
            
            # Add the move to the PGN
            self.current_node = self.current_node.add_variation(move)
            
            # Update previous board array
            self.previous_board_array = new_board_array.copy()
            
            # Get current move number and turn
            move_number = (self.board.fullmove_number - 1) + (1 if self.board.turn == chess.WHITE else 0)
            is_white_turn = not self.board.turn  # After pushing the move, the turn changes
            
            # Format PGN move
            if is_white_turn:
                pgn_move = f"{move_number}. {san_move}"
            else:
                pgn_move = san_move
            
            print(f"Successfully updated board with move: {move}")
            return pgn_move
            
        except Exception as e:
            print(f"Error updating board: {e}")
            print(f"Current FEN: {self.board.fen()}")
            print(f"Attempted move: {move}")
            return None
    
    def get_pgn(self):
        """Return the current game in PGN format"""
        return str(self.game)
        
 # Example usage:
def main():
    # Initialize the analyzer
    analyzer = ChessBoardAnalyzer()
    
    print("---- Example 1: Using standard detection ----")
    # Simulate a series of board states
    # This would be replaced by your computer vision detection
    
    # Create test boards with sequential moves
    test_board = chess.Board()
    
    # Store initial state
    initial_array = analyzer.board_to_array(test_board)
    
    # First move: e4
    test_board.push_san("e4")
    board1_array = analyzer.board_to_array(test_board)
    move1 = analyzer.update_board(board1_array)
    print(f"Detected move: {move1}")
    
    # Second move: e5
    test_board.push_san("e5")
    board2_array = analyzer.board_to_array(test_board)
    move2 = analyzer.update_board(board2_array)
    print(f"Detected move: {move2}")
    
    # Third move: Nf3
    test_board.push_san("Nf3")
    board3_array = analyzer.board_to_array(test_board)
    move3 = analyzer.update_board(board3_array)
    print(f"Detected move: {move3}")
    
    # Get the PGN
    pgn = analyzer.get_pgn()
    print("\nFull PGN:")
    print(pgn)
    

if __name__ == "__main__":
    main()