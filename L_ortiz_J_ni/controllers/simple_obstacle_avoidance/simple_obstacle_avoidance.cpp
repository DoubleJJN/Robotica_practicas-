/**
 * @file    simple_obstacle.cpp
 * @brief   A simple example for avoiding obstacles.
 *
 * @author  Álvaro Castro González <acgonzal@ing.uc3m.es>
 * @date    2023-09
 */

#include "MyRobot.h"

/**
 * @brief Main program.
 */
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
