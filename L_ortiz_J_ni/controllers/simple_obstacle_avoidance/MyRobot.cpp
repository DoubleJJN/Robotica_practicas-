/**
 * @file    MyRobot.cpp
 * @brief   A simple controller for wall following -> Clockwise preference
 * @author  Javier Pastor Fernandez <javpasto@ing.uc3m.es>
 * @date    2021
 */


#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{
    // init default values
    _time_step = 64; // control time step
    
    ds_name[0] = "ds1";
    ds_name[1] = "ds14";
    ds_name[2] = "ds3"; //outer_left_ir
    ds_name[3] = "ds12"; //outer_right_ir
    
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _mode = FORWARD;

    //initialize distance sensors
    for (int ind = 0; ind < NUM_DISTANCE_SENSOR; ind++){
	cout << "Initializing distance sensor: " <<  ds_name[ind] << std::endl;
	_distance_sensor[ind] = getDistanceSensor(ds_name[ind]);
	_distance_sensor[ind]->enable(_time_step);  
    }   
    // get motors
    left_motor = getMotor("left wheel motor");
    right_motor = getMotor("right wheel motor");

    // set position to infinity, to allow velocity control 
    left_motor->setPosition(INFINITY);
    right_motor->setPosition(INFINITY);

}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable devices --> distance sensor
    for (int ind = 0; ind < NUM_DISTANCE_SENSOR; ind++)
    {
	cout << "Disabling distance sensor: " <<  ds_name[ind] <<endl;
	_distance_sensor[ind]->disable();
    }  
    
    _my_compass->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double compass_angle;
    double ir1_val = 0.0, ir14_val = 0.0,
           ir3_val = 0.0, ir12_val = 0.0; // initialize distance sensor values
    
    // Main loop
    while (step(_time_step) != -1) {
            ir1_val = _distance_sensor[0]->getValue();
            ir14_val = _distance_sensor[1]->getValue();
            ir3_val = _distance_sensor[2]->getValue();
            ir12_val = _distance_sensor[3]->getValue();
            
            const double *compass_val = _my_compass->getValues();

            // convert compass bearing vector to angle, in degrees
            compass_angle = convert_bearing_to_degrees(compass_val);

            // print sensor values to console
            cout << "Compass angle (degrees): " << compass_angle << endl;
            
            // print sensors info
            cout << "Sensor frontLeft: " << ir1_val <<endl;
            cout << "Sensor frontRight: " << ir14_val <<endl;
            cout << "Sensor outerLeft: " << ir3_val <<endl;
            cout << "Sensor outerRight: " << ir12_val <<endl;

            // control logic of the robot
            bool wallFront = (ir1_val > DISTANCE_LIMIT || ir14_val > DISTANCE_LIMIT);
            bool wallLeft = (ir3_val > DISTANCE_LIMIT);
            bool wallRight = (ir12_val > DISTANCE_LIMIT);
    
            if (wallFront) {
              _mode = OBSTACLE_AVOID;
              cout <<"Backing up and turning left." << endl;
            } else if (wallLeft || wallRight) {
              _mode = FORWARD;
            } else if (wallLeft && wallFront) 
            } else {
              _mode = FOLLOW_COMPASS;
              cout <<"Go forward." << endl;
            }


            // send actuators commands according to the mode
            switch (_mode){
              case FOLLOW_COMPASS:
                // simple bang-bang control
                if (compass_angle < (DESIRED_ANGLE - 2)) {
                    // turn right
                    left_speed = MAX_SPEED;
                    right_speed = MAX_SPEED - 5;
                }
                else {
                    if (compass_angle > (DESIRED_ANGLE + 2)) {
                        // turn left
                        left_speed = MAX_SPEED - 5;
                        right_speed = MAX_SPEED;
                    }
                    else {
                        // move straight forward
                        cout<<"Moving forward"<<endl;
                        left_speed = MAX_SPEED;
                        right_speed = MAX_SPEED;
                    }
                }
                break;
              case OBSTACLE_AVOID:
                left_speed = -MAX_SPEED / 40.0;
                right_speed = -MAX_SPEED / 6.0; 
                break;
              case TURN_LEFT:
                left_speed = MAX_SPEED - 5;
                right_speed = MAX_SPEED;
                break;
              case TURN_RIGHT:
                left_speed = MAX_SPEED;
                right_speed = MAX_SPEED - 5;
                break;
              case FORWARD:
                left_speed = MAX_SPEED;
                right_speed = MAX_SPEED;
                break;
            default:
                break;
            }

            // set motor velocities
            left_motor->setVelocity(left_speed);
            right_motor->setVelocity(right_speed);

        
        
    }

}
  //////
  
  double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    //Ajustar a rango 0–360
    if (deg < 0)
        deg += 360.0;
        
    return deg;
}

//////