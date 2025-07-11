#include "boardPos.h"
#include "robotControl.h"

// Constructor
BoardPos::BoardPos() {
    initialiseBoard();
    initialiseCapturedBoards();
}

static const std::array<std::array<double, 6>, 64> joints {{
    {{0.951626,-1.0902,1.14072,-1.62198,-1.56842,-0.608655}},
    {{0.893894,-1.25154,1.37397,-1.69375,-1.56838,-0.666388}},
    {{0.824844,-1.40472,1.56954,-1.73596,-1.56835,-0.735438}},
    {{0.741016,-1.55695,1.73752,-1.75152,-1.56833,-0.819267}},
    {{0.637515,-1.713,1.88188,-1.73956,-1.56833,-0.922768}},
    {{0.507315,-1.87641,2.00329,-1.69725,-1.56837,-1.05297}},
    {{0.340252,-2.04926,2.10032,-1.62103,-1.56847,-1.22003}},
    {{0.12182,-2.23115,2.17008,-1.50842,-1.56871,-1.43846}},
    {{1.02895,-1.13657,1.21061,-1.64567,-1.56848,-0.531335}},
    {{0.975987,-1.29909,1.43756,-1.70998,-1.56844,-0.584295}},
    {{0.911737,-1.45678,1.6301,-1.74468,-1.56839,-0.648545}},
    {{0.832226,-1.61687,1.79641,-1.75071,-1.56836,-0.728057}},
    {{0.731408,-1.78509,1.93927,-1.7251,-1.56833,-0.828874}},
    {{0.599614,-1.96676,2.05806,-1.6619,-1.56834,-0.960669}},
    {{0.420288,-2.1669,2.14927,-1.55253,-1.56841,-1.14}},
    {{0.161721,-2.39073,2.20588,-1.38472,-1.56866,-1.3985}},
    {{1.11188,-1.1675,1.25602,-1.66035,-1.56855,-0.448404}},
    {{1.06533,-1.33143,1.47941,-1.7197,-1.56851,-0.49495}},
    {{1.00819,-1.4928,1.67022,-1.74901,-1.56846,-0.552095}},
    {{0.936295,-1.65914,1.8355,-1.74777,-1.56841,-0.623987}},
    {{0.842945,-1.83719,1.97716,-1.71116,-1.56836,-0.717337}},
    {{0.716373,-2.03414,2.09341,-1.63015,-1.56833,-0.843909}},
    {{0.53319,-2.25872,2.17833,-1.49003,-1.56836,-1.02709}},
    {{0.233045,-2.52717,2.21767,-1.26023,-1.56858,-1.32724}},
    {{1.19956,-1.18308,1.27857,-1.6675,-1.56865,-0.360717}},
    {{1.16096,-1.3479,1.50033,-1.72436,-1.56861,-0.399325}},
    {{1.11316,-1.51134,1.69034,-1.75082,-1.56856,-0.44712}},
    {{1.05232,-1.68119,1.85512,-1.74562,-1.5685,-0.507962}},
    {{0.971928,-1.8648,1.99609,-1.70279,-1.56844,-0.588354}},
    {{0.859845,-2.07066,2.11072,-1.6113,-1.56837,-0.700437}},
    {{0.689219,-2.31031,2.1913,-1.4518,-1.56833,-0.871064}},
    {{0.488392,-2.52519,2.21788,-1.26301,-1.56838,-1.07189}},
    {{1.29078,-1.18327,1.27891,-1.66785,-1.56878,-0.269498}},
    {{1.26132,-1.34809,1.50066,-1.72471,-1.56874,-0.298962}},
    {{1.2247,-1.51156,1.69067,-1.75118,-1.56869,-0.335581}},
    {{1.17783,-1.68145,1.85546,-1.74597,-1.56863,-0.382451}},
    {{1.1154,-1.86514,1.99645,-1.70314,-1.56856,-0.444885}},
    {{1.02724,-2.07112,2.11109,-1.6116,-1.56848,-0.533044}},
    {{0.890014,-2.31101,2.19165,-1.45195,-1.56839,-0.670268}},
    {{0.726771,-2.5234,2.21808,-1.26559,-1.56834,-0.833511}},
    {{1.38403,-1.16806,1.25707,-1.66139,-1.56892,-0.176247}},
    {{1.36442,-1.33201,1.48041,-1.72075,-1.56889,-0.195857}},
    {{1.34007,-1.49344,1.67121,-1.75007,-1.56885,-0.220206}},
    {{1.30895,-1.65991,1.83652,-1.74885,-1.56881,-0.251327}},
    {{1.2676,-1.83816,1.97822,-1.71222,-1.56875,-0.292677}},
    {{1.20951,-2.03546,2.09451,-1.63109,-1.56867,-0.350767}},
    {{1.12025,-2.26065,2.17941,-1.49061,-1.56857,-0.440027}},
    {{0.955601,-2.53037,2.21846,-1.25955,-1.56843,-0.604681}},
    {{1.47768,-1.13752,1.21238,-1.64741,-1.56908,-0.0826024}},
    {{1.46805,-1.30006,1.43924,-1.71172,-1.56906,-0.092228}},
    {{1.45617,-1.45783,1.63176,-1.74645,-1.56904,-0.104115}},
    {{1.44109,-1.61809,1.79811,-1.75251,-1.56901,-0.119188}},
    {{1.42131,-1.7866,1.94103,-1.72688,-1.56898,-0.138973}},
    {{1.39407,-1.96873,2.05989,-1.66356,-1.56894,-0.166211}},
    {{1.35383,-2.16964,2.15112,-1.5538,-1.56887,-0.206454}},
    {{1.28677,-2.39481,2.20755,-1.38493,-1.56878,-0.273507}},
    {{1.57011,-1.09154,1.14328,-1.62444,-1.56925,0.00983326}},
    {{1.57005,-1.25287,1.37634,-1.69618,-1.56925,0.0097647}},
    {{1.56996,-1.40614,1.57186,-1.73842,-1.56925,0.00968081}},
    {{1.56986,-1.55856,1.73987,-1.75402,-1.56925,0.00957581}},
    {{1.56972,-1.71493,1.8843,-1.74208,-1.56925,0.00944057}},
    {{1.56954,-1.87882,2.00581,-1.69969,-1.56925,0.00925984}},
    {{1.56929,-2.05242,2.1029,-1.62318,-1.56925,0.00900601}}  
    }
};


void BoardPos::initialiseBoard() {
    for (int i = 0; i < 64; i++) {
        int file = i / 8; //files are columns
        int rank = i % 8; //ranks are rows 

        int flipped_file = 7 - file; // H=0, G=1, ..., A=7
        int flipped_rank = 7 - rank; // 8=0, 7=1, ..., 1=7

        //X: Board spans -150 to +150mm, square centers at odd values
        board[i].position.x = (-0.15 + 0.01875) + 0.0375 * (flipped_file);
        //Y: Board spans 200 to 500mm, square centers at odd values  
        board[i].position.y = (0.13 + 0.01875) + 0.0375 * (flipped_rank); 
        board[i].position.z = 0.047;

        board[i].jointValues.joints = joints[i];
    }
}

void BoardPos::initialiseCapturedBoards() {
    for (int i = 0; i < 16; i++) {

        int row = i / 8;
        int col = i % 8;
        
        //40mm offset between chessboard and array
        whiteCapturedPieces[i].position.x = (-0.150 - 0.040 - 0.075 + 0.01875) + 0.0375 * row;
        whiteCapturedPieces[i].position.y = (0.13 + 0.01875) + 0.0375 * col;
        whiteCapturedPieces[i].position.z = 0.047;
        whiteCapturedPieces[i].full = false;

        blackCapturedPieces[i].position.x = (0.150 + 0.040 + 0.01875) + 0.0375 * row;       
        blackCapturedPieces[i].position.y = (0.13 + 0.01875) + 0.0375 * col;
        blackCapturedPieces[i].position.z = 0.047;
        blackCapturedPieces[i].full = false;
    }
}

bool BoardPos::isPawnPromotion(int destinationIndex) {

    // Check if a pawn has a reached the opposite end of the board
    // check if destination is on the 8th rank (indices 56-63)

    return false;
}

std::vector<int> BoardPos::chessNotationToIndex(const std::string& notation) {
    if (notation.length() != 5) {
        std::cerr << "Invalid chess notation: " << notation << std::endl;
        return {};
    }
    
    char file = std::toupper(notation[0]); // Convert to uppercase (A-H)
    char rank = notation[1]; // Numeric rank (1-8)
    
    char file2 = std::toupper(notation[2]);
    char rank2 = notation[3];

    char value = notation[4];

    if (file < 'A' || file > 'H' || rank < '1' || rank > '8' || file2 < 'A' || file2 > 'H' || rank2 < '1' || rank2 > '8' || (value != '0' && value != '1') ) {
        std::cerr << "Invalid chess notation: " << notation << std::endl;
        return {};
    }
    
    // Calculate array index (A1 is 0, A2 is 1, ..., H8 is 63) - column-major order
    int col = file - 'A';
    int row = rank - '1';
    int col2 = file2 - 'A';
    int row2 = rank2 - '1';
    int taken = value - '0'; //methods converts char into int

    int firstIndex = col * 8 + row;
    int secondIndex = col2 * 8 + row2;

    return {firstIndex, secondIndex, taken};
}



BoardPos::Position3D BoardPos::getCapturedPiecePosition() {
    // Your existing logic for finding empty captured piece slot
    for (int i = 0; i < whiteCapturedPieces.size(); i++) {
        if (whiteCapturedPieces[i].full == false) {
            whiteCapturedPieces[i].full = true;
            return whiteCapturedPieces[i].position;
        }
    }
    throw std::runtime_error("No empty slot found in captured board!");
}

BoardPos::Position3D BoardPos::getBoardPosition(int index) {
    if (index < 0 || index >= 64) {
        throw std::out_of_range("Board index out of range");
    }
    return board[index].position;
}

std::array<double, 6> BoardPos::getJointValue(int index) {
        if (index < 0 || index >= 64) {
        throw std::out_of_range("Board index out of range");
    }
    return board[index].jointValues.joints;
}


