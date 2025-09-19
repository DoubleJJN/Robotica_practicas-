/**
 * @file    MyRobot.cpp
 * @brief   A simple example for maintaining a straight line with the compass.
 *
 * @author  Sara Marqués Villarroya <smarques@ing.uc3m.es>
 * @author  Juan José Gamboa Montero <jgamboa@ing.uc3m.es>
 * @date    2020-10
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{
    // init default values
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
    
    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable devices
    _my_compass->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double compass_angle;

    while (step(_time_step) != -1) {
        // read the sensors
        const double *compass_val = _my_compass->getValues();

        // convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;

        // simple bang-bang control
        if (compass_angle < (DESIRED_ANGLE - 2)) {
            // turn right
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED - 3;
        }
        else {
            if (compass_angle > (DESIRED_ANGLE + 2)) {
                // turn left
                _left_speed = MAX_SPEED - 3;
                _right_speed = MAX_SPEED;
            }
            else {
                // move straight forward
                cout<<"Moving forward"<<endl;
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
            }
        }

        // set the motor position to non-stop moving
        _left_wheel_motor->setPosition(INFINITY);
        _right_wheel_motor->setPosition(INFINITY);
        
        // set the motor speeds
        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
    }
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////
