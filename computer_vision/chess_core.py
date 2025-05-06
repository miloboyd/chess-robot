import numpy as np
import matplotlib.pyplot as plt
import python_chess3 as chs
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from stockfish import Stockfish
from std_msgs.msg import String, Bool
import time
import tkinter as tk


class Chess_Core(Node):
    def __init__(self):
        super().__init__('Chess_Core')

        self.subscription = self.create_subscription(Image, '/camera', self.listener_callback, 10)
        self.create_subscription(Bool, '/move_complete', self.move_done_callback, 10)
        self.publisher = self.create_publisher(String, '/send_move', 10)
        self.bridge = CvBridge()
        self.prev_img = None
        self.current_img = None

        self.game = chs.game() #the actual chess game
        self.board = chs.chess.Board() #temporary board for checking stuff
        self.current_board = []
        self.corners = []

        self.turn = 0
        self.turn_toggle = 0
        self.move_flag = 1 #used to check if the robot has completed its movement 1 means move complete

        self.get_logger().info('Camera subscriber node started.')

                # Set the correct Stockfish binary path here
        STOCKFISH_PATH = "/usr/games/stockfish"  # Change this based on your OS

        self.stockfish = Stockfish(STOCKFISH_PATH)
        self.stockfish.set_depth(18)  # Search depth (higher = stronger but slower)
        self.stockfish.update_engine_parameters({
            "Threads": 2, 
            "Hash": 512,
            "Skill Level": 20  # 0 (weakest) to 20 (strongest)
        })

    def listener_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.prev_img is None:
                self.prev_img = img
            else:
                self.current_img = img
                self.get_logger().info(f"Received image: {msg.width}x{msg.height}")
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def move_done_callback(self, msg):
        if msg.data:
            self.get_logger().info("Robot move confirmed complete.")

    def get_best_move(self, fen: str) -> str:
        """Takes a FEN string and returns the best move."""
        if not self.stockfish.is_fen_valid(fen):
            return "Invalid FEN position."
        
        self.stockfish.set_fen_position(fen)
        best_move = self.stockfish.get_best_move()
        return best_move

    def chess_coord_to_index(self, coord):
        file_to_col = {'a': 0, 'b': 1, 'c': 2, 'd': 3,
                    'e': 4, 'f': 5, 'g': 6, 'h': 7}
        rank_to_row = {'1': 7, '2': 6, '3': 5, '4': 4,
                    '5': 3, '6': 2, '7': 1, '8': 0}  # flipped

        if len(coord) != 2 or coord[0] not in file_to_col or coord[1] not in rank_to_row:
            raise ValueError(f"Invalid coordinate: {coord}")
        
        col = file_to_col[coord[0]]
        row = rank_to_row[coord[1]]
        return (row, col)

    def check_move(self):
        i = 1
        results = ""
        while i ==1:
            if self.current_board is None or len(self.current_board) == 0:
                self.current_board, self.corners  = self.game.analyze_chessboard(self.prev_img, auto_calib=False)
                print(self.current_board)
            else:
                board_array, _ = self.game.analyze_chessboard(self.current_img, auto_calib=False, corners=self.corners)
                #print("Board representation (0=empty, 1=white, 2=black):")

                results = self.game.analyze_binary_board_state(self.game, board_array)

                # Access the analysis results
                if results["detected_move"]:
                    print(f"Move detected: {results['detected_move']}")
                else:
                    print("No valid move detected")

                # The PGN so far
                print(results["pgn"])
                i = 2
        return results['detected_move']
    
    def check_move_sim(self):

        """
        Allows board state to be manually selected instead of using camera feed for testing

        Returns:
            2d array of board position
        """
        i = 1
        results = ""
        while i ==1:
            if self.current_board is None or len(self.current_board) == 0:
                self.current_board = self.manually_select_chess_pieces()
                print(self.current_board)
            else:
                board_array = self.manually_select_chess_pieces()
                #print("Board representation (0=empty, 1=white, -1=black):")

                results = self.game.analyze_binary_board_state(self.game, board_array)

                # Access the analysis results
                if results["detected_move"]:
                    print(f"Move detected: {results['detected_move']}")
                else:
                    print("No valid move detected")

                # The PGN so far
                print(results["pgn"])
                i = 2
        return results['detected_move']

    def update_board(self, move):
        """
        @brief updates the board with a move
        @return if game is over
        """
        self.board.push_san(move)
        board_array = self.game.board_to_colour_array(self.board)
        move_made = self.game.update_board(board_array)
        return self.game.board.is_game_over()

    def get_ai_move(self):
        """
        @brief generates move from stockfish engine using the current board state
        @return move that can be sent to the robot arm in the format <startcoord endcoord goal occupied> eg e2e40
        """
        fen = self.game.board.fen()  # use full FEN, not just board_fen
        movecoord = self.get_best_move(fen)  # e.g., 'e2e4'

        dest_square = movecoord[2:]  # 'e4'
        occupied = self.check_occupied_goal(dest_square)  # returns True or False

        move = movecoord + str(int(occupied))  # e.g., 'e2e41' or 'e2e40'
        return move

    def check_occupied_goal(self, goal):
        """
        @brief checks the current board state against a goal to determine if the target square is occupied or not
        @return true if square is occupied
        """
        array = self.game.board_to_color_array(self.game.board)
        row, col = self.chess_coord_to_index(goal)
        return array[row, col] != 0

    def send_move_to_robot(self, move_str):
        """
        @brief publishes move to robot in format e2e40
        """

        msg = String()
        msg.data = move_str
        self.publisher.publish(msg)
        self.get_logger().info(f'Published move: {move_str}')
        self.move_flag = 0
      
    def check_robot_completed(self):
        return self.move_flag

    def manually_select_chess_pieces(self):
        board_size = 8
        cell_size = 60
        piece_states = [0, 1, -1]  # empty, white, black
        state_colors = {0: 'green', 1: 'white', -1: 'black'}

        root = tk.Tk()
        root.title("Select Chess Pieces")

        board_state = np.zeros((board_size, board_size), dtype=int)
        buttons = [[None for _ in range(board_size)] for _ in range(board_size)]

        def cycle_state(i, j):
            current_state = board_state[i, j]
            next_state_index = (piece_states.index(current_state) + 1) % len(piece_states)
            board_state[i, j] = piece_states[next_state_index]
            buttons[i][j].config(bg=state_colors[board_state[i, j]])

        for i in range(board_size):
            for j in range(board_size):
                btn = tk.Button(root, bg=state_colors[0], width=4, height=2,
                                command=lambda i=i, j=j: cycle_state(i, j))
                btn.grid(row=i, column=j)
                buttons[i][j] = btn

        def finish():
            root.quit()
            root.destroy()

        finish_button = tk.Button(root, text="Finish", command=finish)
        finish_button.grid(row=board_size, column=0, columnspan=board_size, sticky="ew")

        root.mainloop()
        return board_state   
      
    def run_game(self):
        if self.turn == 0:  # player's turn
            print("Player's turn")
            if self.turn_toggle == 1:
                move = self.check_move()
                gameover = self.update_board(move)
                self.turn = 1
                self.turn_toggle = 0
                return gameover

        else:  # robot's turn
            print("Robot's turn")
            move = self.get_ai_move()
            self.send_move_to_robot(move)

            # ⏳ Wait until robot has completed the move
            while not self.check_robot_completed():
                rclpy.spin_once(self, timeout_sec=0.1)

            move = self.check_move()
            gameover = self.update_board(move)
            self.turn = 0
            return gameover

    def start_game(self):
        run = True
        while run:
            run = not self.run_game()

    def run_game_simulated(self):
        self.check_move() #to initialise board state
        if self.turn == 0:  # player's turn
            print("Player's turn")
            if self.turn_toggle == 1:
                move = self.check_move_sim()
                gameover = self.update_board(move)
                self.turn = 1
                self.turn_toggle = 0
                return gameover

        else:  # robot's turn
            print("Robot's turn")
            move = self.get_ai_move()
            self.send_move_to_robot(move)

            # ⏳ Wait until robot has completed the move
            while not self.check_robot_completed():
                rclpy.spin_once(self, timeout_sec=0.1)

            move = self.check_move_sim()
            gameover = self.update_board(move)
            self.turn = 0
            return gameover

    def start_game_sim(self):
        run = True
        while run:
            run = not self.run_game_simulated()

def main(args=None):
    rclpy.init(args=args)
    chess_node = Chess_Core()
    chess_node.start_game_sim()
    chess_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

        

"""
run order
gui params
initialise chess game
start game

human move
turn button pressed
check board position
update board
if game over exit
change turn
send current fen to ai
recieve next move
check if end square occupied
send move command
check board position
update board
if game over exit
change turn

"""