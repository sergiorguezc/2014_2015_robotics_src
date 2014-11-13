/**
 * @file    MyRobot.h
 * @brief   Wall_detector.
 *
 * @author  Sergio Rodríguez Coronel <100081046@alumnos.uc3m.es>
 * @date    2014-11
 */

#include <iostream>

#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 2
#define DISTANCE_LIMIT      100
#define MAX_SPEED           100

class MyRobot : public DifferentialWheels {
    private:
        int _time_step;

        Camera * _forward_camera;

        double _left_speed, _right_speed;

    public:
        // You may need to define your private methods or variables, like
        //  Constructors, helper functions, etc.

        /**
         * @brief Empty constructor of the class.
         * @param _forward_camera
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
