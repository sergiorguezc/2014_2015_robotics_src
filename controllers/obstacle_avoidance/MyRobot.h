/**
 * @file    obstacle_avoidance.cpp
 *
 * @author  Sergio Rodríguez Coronel <100081046@alumnos.uc3m.es>
 * @date    2014-10
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 16
#define DISTANCE_LIMIT      100
#define MAX_SPEED           40
#define DESIRED_ANGLE   45

class MyRobot : public DifferentialWheels {
private:
    int _time_step;

    DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];
    Compass * _my_compass;
    double _left_speed, _right_speed;
    double ir_val[NUM_DISTANCE_SENSOR];

    enum Mode {
        STOP,
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        OBSTACLE_AVOID_L,
        OBSTACLE_AVOID_R,
        WALL_FOLLOWER
    };

    Mode _mode;

public:
    // You may need to define your private methods or variables, like
    //  Constructors, helper functions, etc.

    /**
         * @brief Empty constructor of the class.
         * @param _distance_sensor[NUM_DISTANCE_SENSOR] get value of distance.
         * @param _time_step.
         * @param _left_speed , _right_ speed initialize speed motors
         * @param _my_compass get value of compass
         */
    MyRobot();

    /**
         * @brief Destructor of the class.
         */
    ~MyRobot();

    /**
         * @brief User defined function for initializing and running the template class.
         */
    void run();
    /**
        * @brief User defined function for initializing and running the template class.
        * @param _distance_sensor[NUM_DISTANCE_SENSOR] get value of distance.
        * @return ir_val[NUM_DISTANCE_SENSOR] get value of distance sensor.
        */
    void ReadSensor();

    /**
        * @brief User defined function for initializing and running the template class.
        * @param _my_compass get value of compass
        * @return compass_angle get value of angle.
        */
    double convert_bearing_to_degrees(const double* in_vector);
};
