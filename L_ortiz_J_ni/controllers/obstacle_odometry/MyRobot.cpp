/**
 * @file    MyRobot.cpp
 * @brief   A controller that computes the odometry and uses the info to follow a path
 *          composed by a set of points
 *
 * @author  Juan José Gamboa Montero <jgamboa@ing.uc3m.es>
 * @author  Jesús García Martínez <jesusgar@ing.uc3m.es>
 * @date    2023-12
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

    _x_goal = 16, _y_goal = 0, // final goal
    _theta_goal = atan2((_y_goal - _y),(_x_goal- _x)); // target pose 

    // Motor Position Sensor initialization
    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");

    _left_wheel_sensor->enable(_time_step);
    _right_wheel_sensor->enable(_time_step);   

    // Get robot's compass; initialize it 
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

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
    _left_wheel_sensor->disable();
    _right_wheel_sensor->disable();
}

//////////////////////////////////////////////
// Controller main logic 
void MyRobot::run()
{
    cout << "Goal --> x: " << _x_goal << endl;
    cout << "Goal --> y: " << _y_goal << endl;
    cout << "Goal --> Theta: " << _theta_goal << endl;
    
    // Alligns the odometry initial angle with the current compass value
    _theta = convert_deg_to_rad(-180);
    
    // Overload the goal attributes to follow the first point in the path instead of the final goal
    _active_point = 0;
    _x_goal = _path_goal[_active_point][0];
    _y_goal = _path_goal[_active_point][1];

    while (step(_time_step) != -1) 
    {        
        // Odometry
        this->compute_odometry(); 
        this->print_odometry();
        
        // Movement
        this->go_route();
        
        // If the robot reachoes a partial goal, moves to the next one
        if(this->goal_reached() && (_active_point != _total_points - 1))
        {
            _active_point++;
            _x_goal = _path_goal[_active_point][0]; 
            _y_goal = _path_goal[_active_point][1];
        }
        // If the robot reaches the final goal, stops
        else if(this->goal_reached() && (_active_point == _total_points - 1))       
            break;
    }
}

//////////////////////////////////////////////

void MyRobot::go_route()
{
    // Si el angulo del objetivo es mayor que la tolerancia, gira hacia el objetivo;
    // si no, avanza hacia el objetivo (linea recta)
    if (abs(compute_angle_goal()) > convert_deg_to_rad(DEGREE_TOLERANCE))
        head_goal();    
    else
        move_forward();      
}

//////////////////////////////////////////////

void MyRobot::compute_odometry(bool use_compass)
{
    float new_sl = encoder_tics_to_meters(this->_left_wheel_sensor->getValue());
    float new_sr = encoder_tics_to_meters(this->_right_wheel_sensor->getValue());
    
    float diff_sl = new_sl - _sl; 
    float diff_sr = new_sr - _sr;
    
    _sl = new_sl;
    _sr = new_sr;  
      
    //_theta = *(_my_compass->getValues());
    _x = _x + ((diff_sr + diff_sl) / 2 * cos(_theta + (diff_sr - diff_sl)/(2 * WHEELS_DISTANCE)));
    _y = _y + ((diff_sr + diff_sl) / 2 * sin(_theta + (diff_sr - diff_sl)/(2 * WHEELS_DISTANCE)));    
    
    // To improve the odometry precision
    if (use_compass == true)
      _theta = convert_bearing_to_radians();
    else
    {
        _theta = _theta + ((diff_sr - diff_sl)/WHEELS_DISTANCE);
        
        // To use the same format than the compass [-PI,PI]
        if (_theta <= -M_PI)
            _theta += 2*M_PI;
        else if (_theta >= M_PI)
            _theta -= 2*M_PI;
    }
}  

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_radians()
{
    const double *in_vector = _my_compass->getValues();
    
    // I've changed this line since the angle variation was not correct otherwise
    double rad = atan2(in_vector[2], in_vector[0]);

    return rad;
}      
//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees()
{
    double rad = convert_bearing_to_radians();
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////

double MyRobot::convert_deg_to_rad(double deg)
{    
    double rad = deg * (M_PI / 180.0);
    return rad;
}
//////////////////////////////////////////////

double MyRobot::convert_rad_to_deg(double rad)
{
    double deg = rad * (180.0 / M_PI);
    return deg;
}
//////////////////////////////////////////////

float MyRobot::encoder_tics_to_meters(float tics)
{
    return tics/ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

//////////////////////////////////////////////

void MyRobot::print_odometry()
{
    cout << "x:" << _x << " y:" << _y 
    << " theta:" << _theta 
    << " theta degrees:" << _theta* (180.0 / M_PI) << endl;
}
        
//////////////////////////////////////////////

bool MyRobot::goal_reached()
{
    if (abs(_x_goal-_x) < DISTANCE_TOLERANCE && abs(_y_goal-_y) < DISTANCE_TOLERANCE)
        return true;
  
    return false;
}

//////////////////////////////////////////////

float MyRobot::compute_distance_goal()
{
    float x_target, y_target;
    
    x_target = _x_goal - _x;
    y_target = _y_goal - _y;
    
    double distance = sqrt(pow(x_target, 2) + pow(y_target, 2));
    // cout << "Distance to goal: " << distance << endl;
    return distance; 
}

//////////////////////////////////////////////

float MyRobot::compute_angle_goal()
{
    float x_target, y_target, theta_target;
    
    x_target = _x_goal - _x;
    y_target = _y_goal - _y;
    
    theta_target = atan2(y_target, x_target);
    
    // cout << "Goal orientation: " << convert_rad_to_deg(theta_target) << endl;
    // cout << "Current orientation: " << convert_rad_to_deg(_theta) << endl;
    
    // Proceed to compute the diference
    theta_target -= _theta;
    
    // If the difference is greater than PI, that means it is avoiding the -PI/PI discontinuity
    // to solve this, we sum or substract 2*PI to the angle difference
    if (theta_target < -M_PI)
        theta_target += 2*M_PI;
    else if (theta_target > M_PI)
        theta_target -= 2*M_PI;
    
    // cout << "Angle difference to goal: " << convert_rad_to_deg(theta_target) << endl;
    return theta_target;  
}

//////////////////////////////////////////////

void MyRobot::head_goal()
{    
    // Compute the angle difference with respect to the current goal
    float angle_difference = compute_angle_goal();
    //Here we evaluate whether the rotation is clockwise or counterclockwise
    // turn right
    if (angle_difference < -convert_deg_to_rad(DEGREE_TOLERANCE))
    {
        _left_speed = SLOW_SPEED;
        _right_speed = -SLOW_SPEED;
        
        // Reduces the turning speed when the angle is smaller
        if (angle_difference > -convert_deg_to_rad(DEGREE_TOLERANCE * 2))
        {
            _left_speed = SLOW_SPEED / 5.0;
            _right_speed = (-SLOW_SPEED) / 5.0;
        }
    }
    // turn left
    else if (angle_difference > convert_deg_to_rad(DEGREE_TOLERANCE))
    {
        _left_speed = -SLOW_SPEED;
        _right_speed = SLOW_SPEED;
        
        // Reduces the turning speed when the angle is smaller
        if (angle_difference < convert_deg_to_rad(DEGREE_TOLERANCE * 2))
        {
            _left_speed = (-SLOW_SPEED) / 5.0;
            _right_speed = SLOW_SPEED / 5.0;
        }
    }
    // stop
    else
    {
        _left_speed = 0;
        _right_speed = 0;
    }
            
    // set the motor speeds
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);  
}

//////////////////////////////////////////////

void MyRobot::move_forward()
{
    float distance = compute_distance_goal();
    // If too close, reduces the speed
    if (distance < DISTANCE_TOLERANCE * 10)
    {
        _left_speed = MEDIUM_SPEED;
        _right_speed = MEDIUM_SPEED;
    }
    else
    {
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED;
    }

    // set the motor speeds
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);  
}

//////////////////////////////////////////////

void MyRobot::stop()
{
    _left_speed = 0;
    _right_speed = 0;
 
    // set the motor speeds
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);  
}


    