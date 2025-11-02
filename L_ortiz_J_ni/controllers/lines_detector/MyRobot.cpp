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

    int sum_left = 0, sum_right = 0, sum_front = 0; // Contadores
    unsigned char green = 0, red = 0, blue = 0; // Variables de la intensidad de cada canal
    double percentage_white_left = 0.0;
    double percentage_white_right = 0.0;
    double percentage_white_front = 0.0;

    int yellow_pixels_cont = 0; // Contador de píxeles amarillos en la franja
    double percentage_yellow = 0.0;
    const double YELLOW_THRESHOLD = 5.0; // Umbral de porcentaje

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

    // Definimos los límites de la franja horizontal para detección de amarillo
    int y_start_limit = (int)(image_height_s * 0.5);   // El 50%
    int y_end_limit = (int)(image_height_s * 0.6); // El 60%
    int stripe_area = image_width_s * (y_end_limit - y_start_limit); // Área total de la franja

    while (step(_time_step) != -1) {
        sum_left = sum_right = sum_front = 0;
        yellow_pixels_cont = 0;
        const unsigned char *image_f = _forward_camera->getImage();
        const unsigned char *image_s = _spherical_camera->getImage();

        // Bucle detección de blanco para cada región de la cámara frontal
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

        // Bucle detección de lineas amarillas en cámara esférica
        for (int x = 0; x < image_width_s; x++) {
            for (int y = y_start_limit; y < y_end_limit; y++) { // Solo en la franja definida
                
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

        // --- LÓGICA DE NAVEGACIÓN ---
        if (_lines_cont == 2) {
            _left_speed = 0;
            _right_speed = 0;
            cout << "Línea amarilla final detectada -> me detengo" << endl;
        } else {
            percentage_white_left = (sum_left / (float)(limit_left_f * image_height_f)) * 100;
            percentage_white_right = (sum_right / (float)((image_width_f - limit_right_f) * image_height_f)) * 100;
            percentage_white_front = (sum_front / (float)((limit_right_f - limit_left_f) * image_height_f)) * 100;

            cout << "Left:" << percentage_white_left 
                << "Front:" << percentage_white_front 
                << "Right:" << percentage_white_right << endl;

            // Detección de paredes
            bool wallLeft = (percentage_white_left > 15);
            bool wallFront = (percentage_white_front > 25);
            bool wallRight = (percentage_white_right > 15);
            float aux = percentage_white_left - percentage_white_right;

            // Comportamiento de navegación
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
        // Asignamos velocidades a los motores
        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
        
    }
}
//////////////////////////////////////////////