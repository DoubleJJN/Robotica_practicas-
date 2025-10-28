#include "MyRobot.h"

 /**
  * @brief Main program.
  * @brief This controller implements reactive wall-following behavior
  * @brief The robot explores the world using only the front camera for wall detection
  */
int main(int argc, char** argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}