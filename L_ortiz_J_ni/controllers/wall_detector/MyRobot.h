#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   Controller example for controlling the cameras of the robot.
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
 * @author  Javier Pastor Fernandez <javpasto@ing.uc3m.es>
 * @date    2021-12 
  */

// include dependencies
#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>

using namespace std;
using namespace webots;

// color que queremos detectar (el 255 es demasiado (blanco), nos interesa la intensidad)
// con el 100 nos vale porque no es un blanco perfecto el de la pared
#define THRESHOLD 100

class MyRobot : public Robot {
    public:
        // You may need to define your private methods or variables, like
        //  Constructors, helper functions, etc.

        /**
         * @brief Empty constructor of the class.
         */
        MyRobot();

        /**
         * @brief Destructor of the class.
         */
        ~MyRobot();

        /**
         * @brief Function with the logic of the controller.
         * @param
         * @return
         */
        void run();
        
    private:
        int _time_step; // control time step
        double _left_speed, _right_speed; // velocities

        // Camera sensors
        Camera *_forward_camera;
        Camera *_spherical_camera;

        // Motors
        Motor* _left_wheel_motor;
        Motor* _right_wheel_motor;

};

#endif

