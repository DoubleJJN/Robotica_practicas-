#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   A simple example for computing the odometry while the robot moves straight
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
 * @author  Alvaro Castro-Gonzalez <acgonzal@ing.uc3m.es>
 * @author  Javier Pastor Fernandez <javpasto@ing.uc3m.es>
 * @date    2021-12 
 */

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Compass.hpp>
#include <math.h>

using namespace std;
using namespace webots;

#define MAX_SPEED       10

#define WHEELS_DISTANCE 0.32   //[=] meters
#define WHEEL_RADIUS    0.0825 //[=] meters


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
    void run();

    /**
     * @brief Prints in the standard output the x,y,theta coordinates of the robot.
     */
    void print_odometry();

    /**
     * @brief Checks whether the robot has reached the goal established for this controller.
     * @return true if the robot has reached the goal established for this controller; false otherwise.
     */
    bool goal_reached();
    

private:
    int _time_step;

    // velocities
    double _left_speed, _right_speed;
        
    float _x, _y, _x_goal, _y_goal;   // [=] meters
    float _theta, _theta_goal;   // [=] rad
    
    float _sr, _sl;  // [=] meters

    // Motor Position Sensor
    PositionSensor* _left_wheel_sensor;
    PositionSensor* _right_wheel_sensor;

    
    // Compass sensor
    Compass * _my_compass;

    // Motors
    Motor* _left_wheel_motor;
    Motor* _right_wheel_motor;


	/**
        * @brief Updates the odometry of the robot in meters and radians. The atributes _x, _y, _theta are updated.
        */
    void compute_odometry();
        
 	/**
        * @brief Computes orientation of the robot in degrees based on the information from the compass         * 
        * @return orientation of the robot in degrees 
        */       
    double convert_bearing_to_degrees();
        
};

#endif

