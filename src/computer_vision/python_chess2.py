import numpy as np
import chess
import chess.pgn
from typing import Tuple, Optional, List

class ChessBoardAnalyzer:
    def __init__(self, initial_fen=None):
        """
        Initialize the ChessBoardAnalyzer
        
        Args:
            initial_fen: Optional FEN string to set the initial board state
                         If None, starts with the standard chess starting position
        """
        # Initialize a chess board
        if initial_fen:
            self.board = chess.Board(initial_fen)
        else:
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
        self.previous_board_array = self.board_to_color_array(self.board)
    
    def board_to_color_array(self, board):
        """
        Convert a python-chess board to an 8x8 NumPy array showing only piece colors
        
        Array values:
        0 = empty square
        1 = white piece
        -1 = black piece
        """
        board_array = np.zeros((8, 8), dtype=int)
        
        for square in chess.SQUARES:
            piece = board.piece_at(square)
            if piece:
                # Get rank and file (0-7)
                rank = chess.square_rank(square)
                file = chess.square_file(square)
                
                # Set value based only on color (1 for white, -1 for black)
                value = 1 if piece.color == chess.WHITE else -1
                board_array[7-rank][file] = value
        
        return board_array
    
    def detect_move(self, new_color_array):
        """Detect the move by comparing the previous and new board states with color-only arrays"""
        # Find changed squares
        changed_squares = np.where(self.previous_board_array != new_color_array)
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
            
            # Convert to color array and compare with new_color_array
            temp_array = self.board_to_color_array(temp_board)
            
            # Calculate number of matching positions
            matches = np.sum(temp_array == new_color_array)
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
    
    def update_board(self, new_color_array, forced_turn=None):
        """
        Update the board with a newly detected position and return the PGN move
        
        Args:
            new_color_array: Current observed board state as NumPy array (color only)
            forced_turn: Force a specific turn (chess.WHITE or chess.BLACK)
                         If None, uses the internal turn tracking
        """
        # Use forced turn if provided, otherwise use internal tracking
        if forced_turn is not None:
            actual_turn = forced_turn
            
            # If we're forcing a specific turn, adjust the board's turn if needed
            if actual_turn != self.board.turn:
                print(f"Adjusting turn to {'White' if actual_turn == chess.WHITE else 'Black'}")
                # Create a new board with the same position but different turn
                new_fen_parts = self.board.fen().split()
                new_fen_parts[1] = 'w' if actual_turn == chess.WHITE else 'b'
                new_fen = ' '.join(new_fen_parts)
                self.board = chess.Board(new_fen)
        
        # Detect move
        move = self.detect_move(new_color_array)
        
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
            self.previous_board_array = new_color_array.copy()
            
            # Get current move number and turn
            move_number = (self.board.fullmove_number - 1) + (1 if not self.board.turn else 0)
            is_white_just_moved = self.board.turn == chess.BLACK  # If it's Black's turn now, White just moved
            
            # Format PGN move
            if is_white_just_moved:
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
    
    def infer_turn(self, current_color_array, new_color_array):
        """
        Try to infer which side is moving based on board changes
        
        Args:
            current_color_array: Previous board state (color only)
            new_color_array: Current board state (color only)
        
        Returns:
            chess.WHITE or chess.BLACK based on likely moving side
        """
        # Find which pieces moved
        if np.array_equal(current_color_array, new_color_array):
            # No change detected
            return None
            
        # Check if piece counts changed
        white_before = np.sum(current_color_array == 1)
        white_after = np.sum(new_color_array == 1)
        
        black_before = np.sum(current_color_array == -1)
        black_after = np.sum(new_color_array == -1)
        
        # If white piece count changed, likely white moved
        if white_before != white_after:
            return chess.WHITE
        
        # If black piece count changed, likely black moved
        if black_before != black_after:
            return chess.BLACK
        
        # If piece counts didn't change, try both turns and see which one gives legal moves
        white_board = chess.Board(self.board.fen())
        if white_board.turn != chess.WHITE:
            # Force white turn
            fen_parts = white_board.fen().split()
            fen_parts[1] = 'w'
            white_board = chess.Board(' '.join(fen_parts))
            
        black_board = chess.Board(self.board.fen())
        if black_board.turn != chess.BLACK:
            # Force black turn
            fen_parts = black_board.fen().split()
            fen_parts[1] = 'b'
            black_board = chess.Board(' '.join(fen_parts))
        
        # Try to find moves for white
        white_moves = []
        for move in white_board.legal_moves:
            temp_board = chess.Board(white_board.fen())
            temp_board.push(move)
            temp_array = self.board_to_color_array(temp_board)
            
            # Calculate match score
            match_score = np.sum(temp_array == new_color_array)
            
            if match_score >= 62:  # Allow for some discrepancy
                white_moves.append((move, match_score))
                
        # Try to find moves for black
        black_moves = []
        for move in black_board.legal_moves:
            temp_board = chess.Board(black_board.fen())
            temp_board.push(move)
            temp_array = self.board_to_color_array(temp_board)
            
            # Calculate match score
            match_score = np.sum(temp_array == new_color_array)
            
            if match_score >= 62:  # Allow for some discrepancy
                black_moves.append((move, match_score))
        
        # Determine which side has better moves
        best_white_score = max([score for _, score in white_moves], default=0)
        best_black_score = max([score for _, score in black_moves], default=0)
        
        if best_white_score > best_black_score:
            return chess.WHITE
        elif best_black_score > best_white_score:
            return chess.BLACK
        elif white_moves and not black_moves:
            return chess.WHITE
        elif black_moves and not white_moves:
            return chess.BLACK
        
        # If we can't determine, use the board's current turn
        return self.board.turn
    
    def process_move(self, new_color_array):
        """
        Process a board state change, inferring whose turn it is if necessary
        
        Args:
            new_color_array: Current observed board state as NumPy array (color only)
            
        Returns:
            PGN formatted move notation
        """
        # First, check if there's actually a change
        if np.array_equal(self.previous_board_array, new_color_array):
            print("No board change detected")
            return None
        
        # Infer whose turn it is
        inferred_turn = self.infer_turn(self.previous_board_array, new_color_array)
        
        if inferred_turn is None:
            print("Couldn't infer which side moved")
            return None
            
        print(f"Inferred turn: {'White' if inferred_turn == chess.WHITE else 'Black'}")
        
        # Update the board with the inferred turn
        return self.update_board(new_color_array, forced_turn=inferred_turn)
    
    def get_pgn(self):
        """Return the current game in PGN format"""
        return str(self.game)

def analyze_binary_board_state(analyzer, new_board_array):
    """
    Analyzes a new binary board state (0 for empty, 1 for white, -1 for black)
    and updates the chess board analyzer.
    
    Args:
        analyzer (ChessBoardAnalyzer): An initialized ChessBoardAnalyzer object
        new_board_array (numpy.ndarray): 8x8 array with 0 (empty), 1 (white), -1 (black)
        
    Returns:
        dict: Analysis results containing:
            - detected_move: The detected move in PGN notation
            - current_fen: The current FEN representation of the board
            - board_array: The current board array after the move
            - board_visual: ASCII visual representation of the board
            - pgn: Current PGN of the game
    """
    # Update the board with the new state
    detected_move = analyzer.update_board(new_board_array)
    
    # Create a visual representation of the board
    board_visual = str(analyzer.board)
    
    # Prepare results
    results = {
        "detected_move": detected_move,
        "current_fen": analyzer.board.fen(),
        "board_array": analyzer.previous_board_array.copy(),
        "board_visual": board_visual,
        "pgn": analyzer.get_pgn()
    }
    
    # Print analysis summary
    print(f"Detected move: {detected_move if detected_move else 'None'}")
    print(f"Current FEN: {analyzer.board.fen()}")
    print(f"Current board state:")
    print(board_visual)
    
    return results

# Example usage:
def main():
    # Initialize the analyzer
    analyzer = ChessBoardAnalyzer()
    
    print("---- Using color-only board representation ----")
    # Simulate a series of board states
    test_board = chess.Board()
    
    # Store initial state
    initial_array = analyzer.board_to_color_array(test_board)
    print("Initial board (color array):")
    print(initial_array)
    
    # First move: e4
    test_board.push_san("e4")
    board1_array = analyzer.board_to_color_array(test_board)
    print("\nBoard after e4:")
    print(board1_array)
    move1 = analyzer.update_board(board1_array)
    print(f"Detected move: {move1}")
    
    # Second move: e5
    test_board.push_san("e5")
    board2_array = analyzer.board_to_color_array(test_board)
    print("\nBoard after e5:")
    print(board2_array)
    move2 = analyzer.update_board(board2_array)
    print(f"Detected move: {move2}")
    
    # Third move: Nf3
    test_board.push_san("Nf3")
    board3_array = analyzer.board_to_color_array(test_board)
    print("\nBoard after Nf3:")
    print(board3_array)
    move3 = analyzer.update_board(board3_array)
    print(f"Detected move: {move3}")
    
    # Get the PGN
    pgn = analyzer.get_pgn()
    print("\nFull PGN:")
    print(pgn)
    
    print("\n---- Example with auto turn inference ----")
    # Reset analyzer
    analyzer2 = ChessBoardAnalyzer()
    
    # Create a new test board and make multiple moves
    test_board2 = chess.Board()
    test_board2.push_san("e4")
    test_board2.push_san("e5")
    test_board2.push_san("Nf3")
    
    # Show what happens when we detect this state directly
    # (skipping intermediate states)
    board_array = analyzer2.board_to_color_array(test_board2)
    print("Board after several moves:")
    print(board_array)
    
    # Process using automatic turn inference
    print("Processing with automatic turn inference...")
    move = analyzer2.process_move(board_array)
    print(f"Detected move: {move}")
    
    # Continue the game
    test_board2.push_san("Nc6")
    next_array = analyzer2.board_to_color_array(test_board2)
    print("\nBoard after Nc6:")
    print(next_array)
    
    # Process next move
    next_move = analyzer2.process_move(next_array)
    print(f"Detected move: {next_move}")
    
    # Get the PGN
    pgn2 = analyzer2.get_pgn()
    print("\nFull PGN:")
    print(pgn2)


if __name__ == "__main__":
    main()