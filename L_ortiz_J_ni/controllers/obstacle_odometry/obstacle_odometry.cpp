#include "MyRobot.h"

 /**
  * @brief Main program.
  */
int main(int argc, char** argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
