#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   A controller that computes the odometry and uses the info to follow a path
 *          composed by a set of points
 *
 * @author  Juan José Gamboa Montero <jgamboa@ing.uc3m.es>
 * @author  Jesús García Martínez <jesusgar@ing.uc3m.es>
 * @date    2023-12
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

// muy importantes las velocidades,
// definen probablemente el 60% del comportamiento del robot
#define MAX_SPEED          10
#define MEDIUM_SPEED        5
#define SLOW_SPEED          1

#define DEGREE_TOLERANCE    5      //[=] degrees (more intuitive)
#define DISTANCE_TOLERANCE  0.1    //[=] meters

#define WHEELS_DISTANCE     0.3606 //[=] meters
#define WHEEL_RADIUS        0.0825 //[=] meters

#define ENCODER_TICS_PER_RADIAN 1


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
     * @brief Main logic of the route.
     */
    void go_route();

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
    
    float _path_goal[5][2] = {{-5,0},{-5, 4},{-13,4},{-13,0},{-18,0}};
    int _active_point, _total_points = 5;
    
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
     * @param whether to use or not the compass to compute theta
     */
    void compute_odometry(bool use_compass=false);
        
    /**
     * @brief Computes orientation of the robot in degrees based on the information from the compass         * 
     * @return orientation of the robot in degrees 
     */       
    double convert_bearing_to_degrees();
    
    /**
     * @brief Computes orientation of the robot in radians based on the information from the compass         * 
     * @return orientation of the robot in radians 
     */ 
    double convert_bearing_to_radians();
    
    /**
     * @brief Utility functions to convert degrees in radians and viceversa         
     * 
     * @param the angle in radians or degrees
     * @return angle in degrees or radians 
     */ 
    double convert_rad_to_deg(double rad);
    double convert_deg_to_rad(double deg);
    
    /**
     * @brief Prints in the standard output the x,y,theta coordinates of the robot. 
     * This method uses the encoder resolution and the wheel radius defined in the model of the robot.
     * 
     * @param tics raw value read from an encoder
     * @return meters corresponding to the tics value 
     */
    float encoder_tics_to_meters(float tics);
    
    float compute_distance_goal();
    float compute_angle_goal();
    
    // Movement fuctions
    void head_goal();
    void move_forward();
    void stop();
        
};

#endif

