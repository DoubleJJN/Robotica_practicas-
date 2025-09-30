/**
 * @file    MyRobot.cpp
 * @brief   A simple example for computing the odometry while the robot moves straight
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
 * @author  Alvaro Castro-Gonzalez <acgonzal@ing.uc3m.es>
 * @author  Javier Pastor Fernandez <javpasto@ing.uc3m.es>
 * @date    2021-12
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{

    // init default values
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _x = _y = _theta = 0.0; // robot pose variables 
    _sr = _sl = 0.0; // displacement right and left wheels

    _x_goal = -3, _y_goal = 3;
    _theta_goal = atan2((_y_goal - _y),(_x_goal- _x)); // target pose 

    // Motor Position Sensor initialization
    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");

    _left_wheel_sensor->enable(_time_step);
    _right_wheel_sensor->enable(_time_step);   

    // Get robot's compass; initialize it 
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
    
    _my_gps = getGPS("gps");
    _my_gps->enable(_time_step);

    // Motor initialization
    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");

    // Set motor position to 0 to re-initialize the encoder measurement
    _right_wheel_motor->setPosition(0.0);
    _left_wheel_motor->setPosition(0.0); 

    // Set motor position to infinity to allow velocity control
    _right_wheel_motor->setPosition(INFINITY); 
    _left_wheel_motor->setPosition(INFINITY);

    // Set motor velocity to 0
    _right_wheel_motor->setVelocity(0.0);
    _left_wheel_motor->setVelocity(0.0);



}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // Stop motors
    _left_wheel_motor->setVelocity(0.0);
    _right_wheel_motor->setVelocity(0.0);
    
    // Disable robot's sensors
    _my_compass->disable();
    _my_gps->disable();
    _left_wheel_sensor->disable();
    _right_wheel_sensor->disable();
}

//////////////////////////////////////////////
// Controller main logic 
void MyRobot::run()
{
    // solo queremos la primera iteración
    if (step(_time_step) != -1) 
    {
        _x = _my_gps->getValues()[2];
        _y = _my_gps->getValues()[0];
        _theta = 0.0;
    }
    
    
    cout << "Goal --> x: " << _x_goal << endl;
    cout << "Goal --> y: " << _y_goal << endl;
    cout << "Goal --> Theta: " << _theta_goal << endl;

    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED;

    // set the motor speeds
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);


    while (step(_time_step) != -1) 
    {
        
        this->compute_odometry(); 
        this->print_odometry();
        if(this->goal_reached()) break;

    }
}

//////////////////////////////////////////////
void MyRobot::compute_odometry()
{
  float sr_meters;
  float sl_meters;
  float delta_sr;
  float delta_sl;
  
  sr_meters = WHEEL_RADIUS * _right_wheel_sensor->getValue();
  sl_meters = WHEEL_RADIUS * _left_wheel_sensor->getValue();
  
  delta_sr = sr_meters - _sr;
  delta_sl = sl_meters - _sl;
  
  _x = _x + ((delta_sr+delta_sl)/2 * cos(_theta + ((delta_sr - delta_sl)/(2*WHEELS_DISTANCE))));
  _y = _y + ((delta_sr+delta_sl)/2 * sin(_theta + ((delta_sr - delta_sl)/(2*WHEELS_DISTANCE))));
  _theta = _theta + ((delta_sr - delta_sl)/WHEELS_DISTANCE);
  
  
}        
//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees()
{
    const double *in_vector = _my_compass->getValues();
    
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}
//////////////////////////////////////////////

void MyRobot::print_odometry()
{
  cout << "x:" << _x << " y:" << _y << " theta:" << _theta << endl;
}
        
//////////////////////////////////////////////

bool MyRobot::goal_reached()
{
  // function to be completed by students
  // ...
  
  return false;

}
//////////////////////////////////////////////