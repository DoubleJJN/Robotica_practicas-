/**
 * @file    MyRobot.cpp
 * @brief   Controller example for controlling the cameras of the robot.
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
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
    int sum_left = 0, sum_right = 0, sum_front = 0; //contador
    unsigned char green = 0, red = 0, blue = 0; //variables de la intensidad de cada canal
    double percentage_white_left = 0.0;
    double percentage_white_right = 0.0;
    double percentage_white_front = 0.0;

    // get size of images for forward camera
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = _forward_camera->getHeight();
    cout << "Size of forward camera image: " << image_width_f << ", " <<  image_height_f << endl;

    // definimos variables de cada region de la imagen de la camara frontal
    int limit_left_f = image_width_f * 0.15; // la region izq es la < de width*0.15
    int limit_right_f = image_width_f * 0.85; // la region der es la > de width*0.85
    int img_front_f = image_width_f * 0.70;

    // get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();
    cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;

    while (step(_time_step) != -1) {
        sum_left = 0;
        sum_right = 0;
        sum_front = 0;

        // get current image from forward camera
        const unsigned char *image_f = _forward_camera->getImage(); //imagen mas reciente

        // aqui analizamos la camara entera, los 128x128 pixeles
        // si quisieramos analizar solo una parte, habria que dividir la camara en regiones y analizar cada una
        // por ejemplo, poner un 15%/70%/15% -> pared izq/frontal/der
        // count number of pixels that are white
        // (here assumed to have pixel value > 245 out of 255 for all color components)

        // bucle for por cada region de la camara
        // LEFT
        for (int x = 0; x < limit_left_f; x++) {
            for (int y = 0; y < image_height_f; y++) {
                green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
                red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
                blue= _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

                if ((green > THRESHOLD) && (red > THRESHOLD) && (blue > THRESHOLD)) {
                    sum_left = sum_left + 1;
                }
            }
        }

        percentage_white_left = (sum_left / (float) (image_width_f * 0.15 * image_height_f)) * 100;
        cout << "Percentage of white in forward camera image LEFT: " << percentage_white_left << endl;

        // bucle for por cada region de la camara
        // RIGHT
        for (int x = 0; x > limit_right_f; x++) {
            for (int y = 0; y < image_height_f; y++) {
                green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
                red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
                blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

                if ((green > THRESHOLD) && (red > THRESHOLD) && (blue > THRESHOLD)) {
                    sum_right = sum_right + 1;
                }
            }
        }

        percentage_white_right = (sum_right / (float) (image_width_f * 0.15 * image_height_f)) * 100;
        cout << "Percentage of white in forward camera image RIGHT: " << percentage_white_right << endl;

        // bucle for por cada region de la camara
        // FRONT
        for (int x = 0; (limit_left_f <= x) && (x >= limit_right_f); x++) {
            for (int y = 0; y < image_height_f; y++) {
                green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
                red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
                blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

                if ((green > THRESHOLD) && (red > THRESHOLD) && (blue > THRESHOLD)) {
                    sum_front = sum_front + 1;
                }
            }
        }

        percentage_white_front = (sum_front / (float) (image_width_f * 0.7 * image_height_f)) * 100;
        cout << "Percentage of white in forward camera image FRONT: " << percentage_white_front << endl;

        // turn around slowly
        _left_speed = 5;
        _right_speed = -5;

        // set the motor speeds
        _right_wheel_motor->setVelocity(_right_speed);
        _left_wheel_motor->setVelocity(_left_speed);
    }
}

//////////////////////////////////////////////
