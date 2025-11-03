/**
 * @file    MyRobot.cpp
 * @brief   Controller example for controlling the cameras of the robot.
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
 * @author  Javier Pastor Fernandez <javpasto@ing.uc3m.es>
 * @date    2021-12 
*/

#include "MyRobot.h"
#include <cmath>

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{
    // init default values
    _time_step = 64;
    _left_speed = 0;
    _right_speed = 0;

    // initialize line counter
    _lines_cont = 0;
    _line_detected = false;

    ds_name[0] = "ds1";
    ds_name[1] = "ds14";
    ds_name[2] = "ds3"; //outer_left_ir
    ds_name[3] = "ds12"; //outer_right_ir
    //front
    ds_name[4] = "ds0";
    ds_name[5] = "ds15";
    
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _mode = FORWARD;

    //initialize distance sensors
    for (int ind = 0; ind < NUM_DISTANCE_SENSOR; ind++){
	  cout << "Initializing distance sensor: " <<  ds_name[ind] << std::endl;
	  _distance_sensor[ind] = getDistanceSensor(ds_name[ind]);
	  _distance_sensor[ind]->enable(_time_step);  
    }   

    // get cameras and enable them
    // las dos camaras son de 128x128 pixeles
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);

    // Motor initialization
    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");

    // Set position to infinity to allow velocity control
    _right_wheel_motor->setPosition(INFINITY); 
    _left_wheel_motor->setPosition(INFINITY);

    // Set initial velocity to 0
    _right_wheel_motor->setVelocity(0.0);
    _left_wheel_motor->setVelocity(0.0);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable camera devices
    _forward_camera->disable();
    _spherical_camera->disable();

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
    double ir1_val, ir14_val,
           ir3_val, ir12_val, ir0_val, ir15_val; // initialize distance sensor values

    // color amarillo
    int limite_canal_R = 180;
    int limite_canal_G = 180;
    int limite_canal_B = 100;

    //int sum_left = 0, sum_right = 0, sum_front = 0; // Contadores
    unsigned char green = 0, red = 0, blue = 0; // Variables de la intensidad de cada canal
    //double percentage_white_left = 0.0;
    //double percentage_white_right = 0.0;
    //double percentage_white_front = 0.0;

    int yellow_pixels_cont = 0; // Contador de píxeles amarillos en la franja
    double percentage_yellow = 0.0;
    const double YELLOW_THRESHOLD = 5.0; // Umbral de porcentaje

    // get size of images for forward camera
    //int image_width_f = _forward_camera->getWidth();
    //int image_height_f = _forward_camera->getHeight();
    //cout << "Size of forward camera image: " << image_width_f << ", " <<  image_height_f << endl;

    // definimos variables de cada región de la imagen de la cámara frontal
    //int limit_left_f = image_width_f * 0.25;
    //int limit_right_f = image_width_f * 0.75;

    // get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();
    cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;

    // Definimos los límites de la franja horizontal para detección de amarillo
    int y_start_limit = (int)(image_height_s * 0.5);   // El 50%
    int y_end_limit = (int)(image_height_s * 0.6); // El 60%
    int stripe_area = image_width_s * (y_end_limit - y_start_limit); // Área total de la franja

    while (step(_time_step) != -1) {

        ir1_val = 0.0; 
        ir14_val = 0.0; 
        ir12_val = 0.0; 
        ir3_val = 0.0; 
        ir0_val = 0.0;
        ir15_val = 0.0;
        
        ir1_val = _distance_sensor[0]->getValue();
        ir14_val = _distance_sensor[1]->getValue();
        ir3_val = _distance_sensor[2]->getValue();
        ir12_val = _distance_sensor[3]->getValue();
        ir0_val = _distance_sensor[4]->getValue();
        ir15_val = _distance_sensor[5]->getValue();
            
        //sum_left = sum_right = sum_front = 0;
        yellow_pixels_cont = 0;

        const double *compass_val = _my_compass->getValues();
        //const unsigned char *image_f = _forward_camera->getImage();
        const unsigned char *image_s = _spherical_camera->getImage();

        // convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;

        cout << "Sensor frontLeft: " << ir1_val <<endl;
        cout << "Sensor frontRight: " << ir14_val <<endl;
        cout << "Sensor outerLeft: " << ir3_val <<endl;
        cout << "Sensor outerRight: " << ir12_val <<endl;
        cout << "Sensor 0: " << ir0_val <<endl;
        cout << "Sensor 15: " << ir15_val <<endl;

        // control logic of the robot
        bool wallFrontLeft = (ir1_val > DISTANCE_LIMIT);
        bool wallFrontRight = (ir14_val > DISTANCE_LIMIT);
        bool wallLeft = (ir3_val > DISTANCE_LIMIT);
        bool wallRight = (ir12_val > DISTANCE_LIMIT);
        bool wallFront =(ir0_val > DISTANCE_LIMIT && ir15_val > DISTANCE_LIMIT);
        
        // ------ LÓGICA DE DETECCIÓN DE LÍNEA AMARILLA ------
        for (int x = 0; x < image_width_s; x++) {
            for (int y = y_start_limit; y < y_end_limit; y++) {
                
                red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y); 
                green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
                blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);

                if ((red > limite_canal_R) && (green > limite_canal_G) && (blue < limite_canal_B)) {
                    yellow_pixels_cont++;
                }
            }
        }

        percentage_yellow = (yellow_pixels_cont / (float)stripe_area) * 100;
        cout << "Porcentaje Amarillo en Franja: " << percentage_yellow << "%" << endl;

        // Verificamos si la línea está presente
        bool yellow_line_present = (percentage_yellow > YELLOW_THRESHOLD);

        if (yellow_line_present && !_line_detected) {
            // Se detecta amarillo y no se había detectado previamente -> Nueva línea
            _lines_cont++;
            _line_detected = true; 
            cout << "LÍNEA AMARILLA (NUEVA): Contador = " << _lines_cont << endl;
        } 
        else if (!yellow_line_present && _line_detected) {
            // No se detecta amarillo y el flag estaba activo -> Línea pasada
            _line_detected = false;
            cout << "Línea amarilla PASADA. Flag reiniciado." << endl;
        }

        // ------ LÓGICA DE NAVEGACIÓN CON SENSORES DE DISTANCIA ------
        if (_lines_cont == 2) {
            _left_speed = 0;
            _right_speed = 0;
            cout << "Línea amarilla final detectada -> me detengo" << endl;
        } else if (wallFrontLeft && !wallFrontRight) {
            _mode = TURN_RIGHT;
            cout <<"TURN RIGHT." << endl;
        } else if (!wallFrontLeft && wallFrontRight) {
            _mode = TURN_LEFT;
            cout <<"TURN LEFT." << endl;
        } else if (wallFront) {
            if (wallLeft && !wallRight) {
                _mode = TURN_RIGHT;
                cout << "TURN RIGHT." << endl;
            }
            else if (wallRight && !wallLeft) {
                _mode = TURN_LEFT;
                cout << "TURN LEFT." << endl;
            }
            else if (wallRight && wallLeft) {
                _mode = OBSTACLE_AVOID;
                cout << "Backing up." << endl;
            }
            else if (!wallRight && !wallLeft) {
                _mode = TURN_LEFT;
                cout << "TURN LEFT (default)." << endl;
            }
            else {
                _mode = FOLLOW_COMPASS;
                cout <<"Go forward." << endl;
            }
        } else if (wallLeft || wallRight) {
            _mode = FORWARD;
        } else if (wallFrontLeft && wallLeft && !wallFrontRight && !wallRight) {
            _mode = TURN_RIGHT;
        } else if (!wallFrontLeft && !wallLeft && wallFrontRight && wallRight) {
            _mode = TURN_LEFT;
        } else if (wallLeft && wallFront) {
            _mode = TURN_RIGHT;
            cout <<"TURN RIGHT." << endl;
        } else if (wallRight && wallFront) {
            _mode = TURN_LEFT;
            cout <<"TURN LEFT." << endl;
        } else {
            _mode = FOLLOW_COMPASS;
            cout <<"Go forward." << endl;
        }

        // send actuators commands according to the mode
        switch (_mode){
            case FOLLOW_COMPASS:
                // simple bang-bang control
                if (compass_angle < (DESIRED_ANGLE_GO - 2)) {
                    // turn right
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED - 5;
                }
                else if (compass_angle > (DESIRED_ANGLE_GO + 2)) {
                    // turn left
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED;
                }
                else {
                    // move straight forward
                    cout<<"Moving forward"<<endl;
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED;
                }
                break;
            case OBSTACLE_AVOID:
                _left_speed = (-10);
                _right_speed = (-5);
                break;
            case TURN_LEFT:
                _left_speed = MAX_SPEED - 5;
                _right_speed = MAX_SPEED;
                break;
            case TURN_RIGHT:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED - 5;
                break;
            case FORWARD:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
                break;
            default:
                break;
        }
        // Asignamos velocidades a los motores
        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
        
    }
}
///////////////////////////////////////////////
double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    //Ajustar a rango 0–360
    if (deg < 0)
        deg += 360.0;
        
    return deg;
}
//////////////////////////////////////////////