import numpy as np
import matplotlib.pyplot as plt
import python_chess2 as chs
import chessboard_analyser as analyser
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class Chess_Core(Node):
    def __init__(self):
        super().__init__('Chess_Core')

        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.prev_img = None
        self.current_img = None

        self.game = chs.game()
        self.board = chs.chess.Board()
        self.turn = 0
        self.turn_toggle = 0

        self.get_logger().info('Camera subscriber node started.')

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

    def check_move(self):
        return analyser.main(self.prev_img, self.current_img)

    def update_board(self, move):
        self.board.push_san(move)
        board_array = self.game.board_to_colour_array(self.board)
        move_made = self.game.update_board(board_array)
        return self.game.board.is_game_over()

    def send_fen_to_ai(self):
        
        return 1

    def check_occupied_goal(self):
        return 0

    def send_move_to_robot(self):
        return 0

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
            self.send_fen_to_ai()
            self.check_occupied_goal()
            self.send_move_to_robot()
            move = self.check_move()
            gameover = self.update_board(move)
            self.turn = 0
            return gameover

    def start_game(self):
        run = True
        while run:
            run = not self.run_game()


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