# Chess Robot

An autonomous chess system featuring a robotic arm to play a competitive match against a human opponent. 

## Project Overview

This project implements an autonomous chess-playing system using a UR3e robotic arm equipped with an RG2 gripper. Leveraging ROS 2, MoveIt and vision-based feedback, the robot plays full games of chess against a human opponent. The robot identifies piece positions, plans its moves, and executes them through precise manipulation, while maintaining safe and reliable motion within the workspace. 

### Key Features

- **UR3e Arm + RG2 Gripper** integration for precise manipulation
- **Real-time chess logic** using stockfish for legal move handling
- **MoveIt** for safe and smooth pick-and-place path planning
- **Vision system** for identifying board state and validating piece positions
- **Modular codebase** with separate nodes for perception, planning, and execution

### Demo

Watch the UR3e robot play a snippet of chess autonomously:

[Robot playing chess](https://youtube.com/shorts/CKr70euElMs?feature=share)

[Robot playing chess](https://youtube.com/shorts/n04GHHTmyB4?feature=share)
