# Robothon2023 RoboPig

## Generell Infos
### Used Hardware
Required:
- UR5e
- Robotiq HandE with rubber tips
- 3Dprinted hook

Optional:
- IDS camera (for board detection)
- Intel Realsense D435i with 3Dprinted holder facing -y of tcp (for slider task)
- googly eyes

### UR5e Settings
- TCP
    - X: 0mm
    - Y: 0mm
    - Z: 148mm
- Payload
    - With Realsense
        - Mass: 1.35kg
        - CX: 3mm
        - CY: 2mm
        - CZ: 50mm
    - Without Realsense
        -Mass: 1.06kg
        - CX: 0mm
        - CY: 0mm
        - CZ: 59mm

## How to install dependencies
### Json
`sudo apt install nlohmann-json3-dev`
### cgal
`sudo apt install libcgal-dev`
### ur_rtde
https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html#linux-ubuntu-and-macos

```
git clone https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde
git checkout v1.5.5
git submodule update --init --recursive
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make -j
sudo make install
```
## How to start
1. `roslaunch robothon2023 taskboard_core.launch`
    - make sure that the robot ip in the launchfile is correct
2. `rosrun robothon2023 robothon2023_touch_detect`
    - !!Attention!! robot is immediately aligning to z 
    - touch so that +y of tcp looks away from taskboard.
    - touch longside without cable holder
    - touch shortside next to probe
    - touch top of the board
    - move robot away
    - Confirm position in rviz
3. `rosrun robothon2023 robothon2023_taskboard`
    - !!WARNING!! robot will move immediately to home position
3. follow shown instructions
## How to Log data
1. uncommend 2x call_log() in task_board_scheduler.cpp 
2. change path in task_board_tasks.cpp

After each run change filename in first call_log() in task_board_scheduler.cpp

