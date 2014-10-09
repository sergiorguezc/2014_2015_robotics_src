/**
 * @file    main_template.cpp
 * @brief   A template for webots projects.
 *
 * @author  Name Surname <nick@alumnos.uc3m.es>
 * @date    2014-07
 */

#include "MyRobot.h"

// This is the main program of your controller.
// It creates an instance of your Robot subclass, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

/**
 * @brief Main program.
 */
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
