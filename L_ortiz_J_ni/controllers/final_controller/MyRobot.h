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
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>

using namespace std;
using namespace webots;

#define THRESHOLD 80
#define MAX_SPEED 8.0
#define MEDIUM_SPEED 3.0
#define MIN_SPEED 1.0

#define NUM_DISTANCE_SENSOR 6
#define DISTANCE_LIMIT 200
#define DESIRED_ANGLE_GO 250
#define DESIRED_ANGLE_RETURN 70

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

        double convert_bearing_to_degrees(const double* in_vector);

        /**
         * @brief Function with the logic of the controller.
         * @param
         * @return
         */
        void run();
        
    private:
        // working modes
        enum  Mode {
            FORWARD,
            TURN_LEFT,
            TURN_RIGHT,
            OBSTACLE_AVOID,
            FOLLOW_COMPASS
        } _mode;

        int _time_step; // control time step
        double _left_speed, _right_speed; // velocities

        // Camera sensors
        Camera *_forward_camera;
        Camera *_spherical_camera;

        // Motors
        Motor* _left_wheel_motor;
        Motor* _right_wheel_motor;

        DistanceSensor* _distance_sensor[NUM_DISTANCE_SENSOR];
        const char *ds_name[NUM_DISTANCE_SENSOR];
        
        Compass *_my_compass;

        // Counter for yellow line detection
        int _lines_cont;
        bool _line_detected;

};

#endif