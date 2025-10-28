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
}

//////////////////////////////////////////////

void MyRobot::run()
{
    // color amarillo
    int limite_canal_R = 180;
    int limite_canal_G = 180;
    int limite_canal_B = 100;

    int sum_left = 0, sum_right = 0, sum_front = 0; //contadores
    int lines_cont = 0; //contador lineas amarillas
    unsigned char green = 0, red = 0, blue = 0; //variables de la intensidad de cada canal
    double percentage_white_left = 0.0;
    double percentage_white_right = 0.0;
    double percentage_white_front = 0.0;

    // get size of images for forward camera
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = _forward_camera->getHeight();
    cout << "Size of forward camera image: " << image_width_f << ", " <<  image_height_f << endl;

    // definimos variables de cada región de la imagen de la cámara frontal
    int limit_left_f = image_width_f * 0.25;
    int limit_right_f = image_width_f * 0.75;

    // get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();
    cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;

    while (step(_time_step) != -1) {
        sum_left = sum_right = sum_front = 0;
        const unsigned char *image_f = _forward_camera->getImage();

        // bucle detección de blanco para cada región de la cámara frontal
        for (int x = 0; x < image_width_f; x++) {
            for (int y = 0; y < image_height_f; y++) {
                red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
                green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
                blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

                if ((red > THRESHOLD) && (green > THRESHOLD) && (blue > THRESHOLD)) {
                    if (x < limit_left_f)
                        sum_left++;
                    else if (x > limit_right_f)
                        sum_right++;
                    else
                        sum_front++;
                }
            }
        }

        // bucle detección de lineas amarillas en cámara esférica
        for (int x = 0; x < image_width_s; x++) {
            for (int y = 0; y < image_height_s; y++) {
                red = _spherical_camera->imageGetRed(image_f, image_width_s, x, y);
                green = _spherical_camera->imageGetGreen(image_f, image_width_s, x, y);
                blue = _spherical_camera->imageGetBlue(image_f, image_width_s, x, y);

                // acción si detectamos el color amarillo en la parte inferior de la cámara esférica
                if ((red > limite_canal_R) && (green > limite_canal_G) && (blue < limite_canal_B)) {
                    if (y < image_height_s * 0.5) {
                        cout << "Color amarillo detectado en cámara esférica (parte inferior)" << endl;
                        lines_cont++;
                    }
                }
            }
        }

        percentage_white_left = (sum_left / (float)(limit_left_f * image_height_f)) * 100;
        percentage_white_right = (sum_right / (float)((image_width_f - limit_right_f) * image_height_f)) * 100;
        percentage_white_front = (sum_front / (float)((limit_right_f - limit_left_f) * image_height_f)) * 100;

        cout << "Left:" << percentage_white_left 
            << "Front:" << percentage_white_front 
            << "Right:" << percentage_white_right << endl;

        cout << "Líneas amarillas detectadas: " << lines_cont << endl;

        // detección de paredes
        bool wallLeft = (percentage_white_left > 15);
        bool wallFront = (percentage_white_front > 25);
        bool wallRight = (percentage_white_right > 15);
        float aux = percentage_white_left - percentage_white_right;

        // detección de líneas amarillas
        if (lines_cont == 2) {
            _left_speed = 0;
            _right_speed = 0;
            cout << "Línea amarilla final detectada -> me detengo" << endl;
        }
        else {
            // comportamiento de navegación
            if (wallFront) {
                if (fabs(aux) <= 5) {
                    _left_speed = -MEDIUM_SPEED;
                    _right_speed = MEDIUM_SPEED;
                } else {
                    if (aux < 0) {
                        _left_speed = MIN_SPEED;
                        _right_speed = MEDIUM_SPEED;
                        cout << "Pared delante -> giro izq" << endl;
                    } else {
                        _left_speed = MEDIUM_SPEED;
                        _right_speed = MIN_SPEED;
                        cout << "Pared delante -> giro der" << endl;
                    }
                }
            } else if (wallLeft) {
                _left_speed = MEDIUM_SPEED;
                _right_speed = MIN_SPEED;
                cout << "Pared izquierda -> me alejo (derecha)" << endl;
            } else if (wallRight) {
                _left_speed = MIN_SPEED;
                _right_speed = MEDIUM_SPEED;
                cout << "Pared derecha -> me alejo (izquierda)" << endl;
            } else {
                _left_speed = MEDIUM_SPEED;
                _right_speed = MEDIUM_SPEED;
                cout << "Camino libre -> avanzo" << endl;
            }
        }
        // asignamos velocidades a los motores
        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
    }
}
//////////////////////////////////////////////