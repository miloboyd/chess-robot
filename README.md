# rs2
rs2 pick and place robotics project

How to run the game:

    ros2 run rs2 chess_core.py

    this will launch the game, initialise the board and start the game with the player being white.
    it is currently configured in sim mode which doesn't use camera inputs for piece detection, insead the user
    must manually enter the piece positions using a gui. click the squares to toggle between white, black and empty.
    
    If not using robot yet.
    the code will wait for a response from the robot indicating that it has completed its move, if you don't have the robot
    working then send this command to a terminal window and the code will progress.

    ros2 topic pub --once /move_complete std_msgs/Bool '{data: true}'

    you will then need to manually enter the new piece positon as entered by the ai in the CLI as camera is diabled.

    the program will then loop until game finished.

Robot node requirements:

    The Chess_core node will send the AI move to the /send_move topic in the format of {starting COORD Finish COORD goal occupied} {eg e2e40}
    it will then wait for a response in the /move_complete topic of true to indicate that the robot has stopped moving.

Camera integration:
    the camera has not been tested yet and therefore has not been fully integrated. I will include instructions on how to use it once it is tested.
    Eventually it will be toggled on using a rosarg.


Equipment Included
4x chessboard quadrants
16x custom white chess pieces
16x custom black chess pieces
1x alignment board

Board Setup:
Place alignment board on the surface of the UR3e bench with corners aligned for future measurements. 
Place all 4 quadrants on the alignment board. Ensure markings below the board follow order so the board is fitted correctly.
Place white chess pieces on board ensuring the Queen is placed on white grid.
Repeat step 3 for black pieces ensuring the black Queen is placed on the black grid.



Ai Subsystem

Overview:
The Ai subsystem is responsible for interpreting the chessboard state and generating optimal moves for the robot using the Stockfish chess engine. This module receives FEN strings from the vision subsystem, processes them and returns a valid move in under two seconds.

Inputs and Outputs:
Input: FEN string (Forsyth-Edwards Notation) from the vision system (Sean’s Subsystem)

Process: Stockfish generates the best move based on the current board state

Output: Valid move in algebraic notation -> passed to the robot manipulation subsystem (Milo’s subsystem)
