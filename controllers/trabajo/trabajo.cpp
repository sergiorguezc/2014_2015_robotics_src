/**
 * @file    obstacle_avoidance.cpp
 * @brief   Obstacle_avoidance.
 *
 * @author  Carlos Hernandez Paz <100292793@alumnos.uc3m.es>
 * @date    2014-11-11
 */

#include "MyRobot.h"

// This is the main program of the controller.
// It creates an instance of your Robot subclass, launches its
// function(s) and destroys it at the end of the execution.
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
