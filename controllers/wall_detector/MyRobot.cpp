/**
 * @file    MyRobot.cpp
 * @brief   Wall_detector.
 *
 * @author  Sergio Rodríguez Coronel <100081046@alumnos.uc3m.es>
 * @date    2014-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);

}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _forward_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    unsigned char green = 0, red = 0, blue = 0;
    double percentage_white = 0.0;


    // Get size of images for forward camera
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = _forward_camera->getHeight();
    cout << "Size of forward camera image: " << image_width_f << ", " <<  image_height_f << endl;

    while (step(_time_step) != -1) {


        // Get current image from forward camera
        const unsigned char *image_f = _forward_camera->getImage();
        int sum = 0;

        // Count number of pixels that are white
        // (here assumed to have pixel value > 245 out of 255 for all color components)
        for (int x = 0; x < image_width_f; x++) {
            for (int y = 0; y < image_height_f; y++) {
                green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
                red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
                blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

                // Always looks white
                if ((green > 245) && (red > 245) && (blue > 245)) {
                    percentage_white = (sum / (float) (image_width_f * image_height_f)) * 100;

                    if(percentage_white > 88){

                        cout << " color blanco detectado " << endl;
                    }
                    else{
                        sum++;
                    }

                }
            }
        }



        // Turn around slowly
        _left_speed = 5;
        _right_speed = -5;

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////
