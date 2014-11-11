/**
 * @file    MyRobot.cpp
 * @brief  obstacle_odometry.
 *
 * @author  Sergio Rodríguez Coronel <10081046@alumnos.uc3m.es>
 * @date    2014-07
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED       100
#define DESIRED_ANGLE   45

class MyRobot : public DifferentialWheels {
private:
    int _time_step;

    Compass * _my_compass;
    double _left_speed, _right_speed;
    double _posicion_final;
    double _velocidad;
    double  _encoder_right;
    double  _encoder_left;
    double _Radio;

    enum Mode {
        STOP,
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT
            };

    Mode _mode;

public:
    /**
         * @brief Empty constructor of the class.
         * @param _time_step.
         * @param _left_speed , _right_ speed initialize speed motors
         * @param _my_compass get value of compass
         * @param Encoder
         */
    MyRobot();

    /**
         * @brief Destructor of the class.
         * @param encoder
         * @return encoders value
         */
    ~MyRobot();

    /**
         * @brief User defined function for initializing and running the template class.
         */
    void run();



    /**
          * @brief An example for converting bearing vector from compass to angle (in degrees).
          */
    double convert_bearing_to_degrees(const double* in_vector);

};
