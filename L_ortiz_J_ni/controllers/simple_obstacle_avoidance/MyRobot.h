#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   A simple controller for avoiding obstacles
 * @author  Álvaro Castro González <acgonzal@ing.uc3m.es>
 * @date    2023
 */


// include dependencies
#include <iostream>
#include <limits>
#include <math.h>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>

using namespace std;
using namespace webots;


#define NUM_DISTANCE_SENSOR 4
#define DISTANCE_LIMIT 250
#define DESIRED_ANGLE 290
#define MAX_SPEED 7
        
class MyRobot : public Robot {
    public:
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
         
         double convert_bearing_to_degrees(const double* in_vector);
         void run();


    private:
        // working modes
        enum  Mode {
            STOP,
            FORWARD,
            TURN_LEFT,
            TURN_RIGHT,
            OBSTACLE_AVOID,
            FOLLOW_COMPASS
        } _mode;
        
        // The time step
        int _time_step;
        
        // velocities
        double left_speed, right_speed;

        // Motors
        Motor *left_motor;
        Motor *right_motor;

        DistanceSensor* _distance_sensor[NUM_DISTANCE_SENSOR];
        const char *ds_name[NUM_DISTANCE_SENSOR];
        
        Compass *_my_compass;

        
};

#endif

