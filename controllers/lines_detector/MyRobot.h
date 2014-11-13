/**
 * @file    MyRobot.h
 * @brief   lines_detector.
 *
 * @author  Sergio Rodríguez Coronel <100081046@alumnos.uc3m.es>
 * @date    2014-11
 */

#include <iostream>
#include <math.h>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 2
#define DISTANCE_LIMIT      50
#define MAX_SPEED           100

class MyRobot : public DifferentialWheels {
private:
    int _time_step;

    Camera * _spherical_camera;
    DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];

    double _left_speed, _right_speed;
    double ir_val[NUM_DISTANCE_SENSOR];
    double percentage_yellow;

    enum Mode {
        STOP,
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT
    };

    Mode _mode;


public:
    // You may need to define your private methods or variables, like
    //  Constructors, helper functions, etc.

    /**
     * @brief Empty constructor of the class.
     * @param _spherical_camera
     * @param _right_speed , _left_speed initialize speed motors
     * @param _time_step.
     */
    MyRobot();

    /**
         * @brief Destructor of the class.
         */
    ~MyRobot();

    /**
         * @brief User defined function for initializing and running the template class.
         * @return _right_speed , _left_speed
         */
    void run();

};
