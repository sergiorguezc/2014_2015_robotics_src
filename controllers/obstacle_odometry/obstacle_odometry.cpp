/**
 * @file    MyRobot.h
 * @brief   obstacle_odometry.
 *
 * @author  Sergio Rodríguez Coronel <10081046@alumnos.uc3m.es>
 * @date    2014-10
 */

#include "MyRobot.h"

/*
 * Main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
