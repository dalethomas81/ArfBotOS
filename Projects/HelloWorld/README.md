# Hello World
If you have ArfBotOS running and want to test out the robot, follow the instructions below to create your first program. Additionally, you will need to have tuned the drives and mastered the robot before proceeding. Please follow the relevant procedure located in the sub-folders of the [Robots](/Robots) directory. For the AR4 robot, those documents are located [here](/Robots/AR4).

## HMI
ArfBotOS host a web-based HMI to make it convenient to program and control from any device. If you followed the installation procedure correctly, the website for ArfBotOS should be http://arfbot:8080/webvisu.htm

## Homing
1. Navigate to the the HMI and enable the robot by pressing the *Enable* button on the top left. You should hear the drives enable.

2. Make sure the robot has enough space to move and home the robot by pressing the *Home* button. Once the robot finishes homing the green *Homed* light will illuminate. 

3. Once the robot is homed it should end up with all axis at their 0 degree position.

## Jogging
1. Navigate to the *JOGGING* screen and notice the *Pose* buttons on the left side. There are 5 dedicated position registers for storing pose positions. Two of these positions are initialized when ArfBotOS boots for the first time. Press **Pose 1** to move the robot. *Note: the pose positions can be found on the PROGRAMS screen in Stored Positions 34-38*
2. Enable the jog function from the [SMC_GroupJog2](https://help.codesys.com/webapp/oRbtGm-DGxt4wWfMHA8h3H12Jpw%2FSMC_GroupJog2;product=SM3_Robotics;version=4.5.1.0) faceplate by pressing the *Enable* button. The *Busy* light will illuminate indicating the function block is ready.
3. Switch to the **TCS** (Tool Center Point) coordinate system using the *CoordSystem* dropdown.
4. Press the *Forward[0]* button to move the TCS in the *X+* position. See the list below for the other axes.
- `Forward[0]` **X+**
- `Forward[1]` **Y+**
- `Forward[2]` **Z+**
- `Forward[3]` **P+** (pitch)
- `Forward[4]` **Y+** (yaw)
- `Forward[5]` **R+** (roll)
*Note: if you are unfamiliar with Euler angles, study [this](https://en.wikipedia.org/wiki/Euler_angles) page. Additionally, search Youtube and other sites on information about 6-axis robot coordinate systems.