For Computer vision folder:

make sure you have opencv, numpy, matplotlib installed and maybe something else idk, the compiler will let you know.
if using vscode make sure you are using the correct interpreter or you'll get a file not found error (the shell where opencv is downloaded, if you don't know chatgpt it)
finally when you try to run the script make sure you are in the computer vision directory in the terminal window or it won't find the images.

Directories:
- pieces: a collection of chess pieces to test contour detection with
- pawns: a collection of pawns, easier testing
- chessboards: bit self expanatory ay
- troubleshooting: a place to put images that don't fit into the other folders to be used for troubleshooting specific cases


Scripts:
- tbh you can pretty much ignore most of them except for chessboard_analyser.py which is the latest and best one that analyses the whole board.
 fairly consistent with pieces but not with blank spaces yet. its doing it because the squares aren't cropping perfectly and its picking up a tiny bit of an
 adjacent square and assuming its a piece, triggering the piece colour identification part of the code. the function returns an 8x8 array of the board that represents 
 the positions of each piece. 0=empty, 1=white, 2=black. next step is to compare 2 different board positions and figure out what moved, probably using an xor operation then
 translating that to chess move.


- square_processing has functions in it that only analyse a specific square, good for testing with pieces and pawns and for troubleshooting individual squares that are giving false readings.
will show the contour trace on the image.