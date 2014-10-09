/**
 * @file    main_template.cpp
 * @brief   A template for webots projects.
 *
 * @author  Name Surname <nick@alumnos.uc3m.es>
 * @date    2014-07
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  _my_compass = getCompass("compass");
    //  _my_compass->enable(_time_step);

    // Always you need to specify the time_step, usually 32 or 64.
    _time_step = 64;
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // Enter here exit cleanup code such as
    // _my_compass->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    // Main loop:
    // Perform simulation steps of 64 milliseconds
    // and leave the loop when the simulation is over
    while (step(_time_step) != -1) {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  const double *compass_val = _my_compass->getValues();

        // Process sensor data here

        // Enter here functions to send actuator commands, like:
        //  setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////
