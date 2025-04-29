import numpy as np
import matplotlib.pyplot as plt
import python_chess2 as chs
import chessboard_analyser as analyser
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Chess_Core(Node):
    def __init__(self):
        super().__init__('Chess_Core')

    def run_game(self, analyser, move_detector, board):
        running = True
        winner = ""

        initial_array = move_detector.board_to_color_array(board)
        



        return running, winner

    def main(self):
        run = False
        board_analyser = chs.ChessBoardAnalyzer()  # Assuming this is correct
        move_detector = chs.ChessBoardAnalyzer()  # Verify this is the correct class
        board = chs.chess.Board()  # Ensure this is defined in chs

        # if run topic == true, set run to True (implement ROS2 topic logic here)

        while run:
            running, winner = self.run_game(board_analyser, move_detector, board)
