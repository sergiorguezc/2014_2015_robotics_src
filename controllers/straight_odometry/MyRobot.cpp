/**
 * @file    MyRobot.cpp
 * @brief  straight_odometry.
 *
 * @author  Sergio Rodríguez Coronel <10081046@alumnos.uc3m.es>
 * @date    2014-07
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
    // Initialize _posicion_final and _Radio.
    _posicion_final = 0.0;
    _Radio = 0.0825;

    enableEncoders(_time_step);



}


//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _my_compass->disable();
    disableEncoders();

}

//////////////////////////////////////////////

void MyRobot::run()
{
    double compass_angle;

    while (step(_time_step) != -1) {

        // Read the sensors
        const double *compass_val = _my_compass->getValues();
        _encoder_right = getRightEncoder();
        _encoder_left = getLeftEncoder();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;

        _posicion_final = _encoder_right/1000 *0.0825;// calculate the next position
        cout << "posicion :" << _posicion_final << endl;

        // control condition
        if(_posicion_final < 0.087){

            if (compass_angle < (DESIRED_ANGLE - 2)) {
                // Turn right
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED - 15;
            }
            else {
                if (compass_angle > (DESIRED_ANGLE + 2)) {
                    // Turn left
                    _left_speed = MAX_SPEED - 15;
                    _right_speed = MAX_SPEED;
                }
                else {
                    // Move straight forward
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED;
                }
            }


        }
        else{
            _left_speed = 0.0;
            _right_speed = 0.0;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
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
