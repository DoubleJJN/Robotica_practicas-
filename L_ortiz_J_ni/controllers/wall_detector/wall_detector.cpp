#include "MyRobot.h"

 /**
  * @brief Main program.
  * @brief This controller implements reactive wall-following behavior
  * @brief The robot explores the world using only the front camera for wall detection
  */
int main(int argc, char** argv)
{
    MyRobot* my_robot = new MyRobot();
    
    cout << "Starting reactive wall-following behavior..." << endl;
    cout << "Robot will explore the world by following walls" << endl;
    cout << "Using only front camera for wall detection" << endl;
    cout << "No distance sensors, compass, GPS, or target points needed" << endl;
    cout << "Pure reactive behavior based on immediate visual input" << endl;
    cout << "=========================================" << endl;

    my_robot->run();

    delete my_robot;

    return 0;
}