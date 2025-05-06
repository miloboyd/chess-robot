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

    The camera has not been tested yet and therefore has not been fully integrated. I will include instructions on how to use it once it is tested.
    Eventually it will be toggled on using a rosarg.
