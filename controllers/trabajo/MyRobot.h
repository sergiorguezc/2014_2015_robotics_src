/**
 * @file    MyRobot.h
 * @brief   Obstacle_avoidance.
 *
 * @author  Carlos Hernandez Paz <100292793@alumnos.uc3m.es>
 * @date    2014-11-11
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>
#include<string>
#include<sstream>
#include<time.h>
#include<stdio.h>
#include<ctime>



using namespace std;
using namespace webots;

//definition of constant variables

#define NUM_DISTANCE_SENSOR 16
#define DISTANCE_LIMIT      60
#define MAX_SPEED           60
#define DESIRED_ANGLE       55

//definition of the class MyRobot

class MyRobot : public DifferentialWheels {

    //declaration of the privates variables
private:

    int _time_step;
    Compass * _my_compass;
    DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];
    double ir_val[NUM_DISTANCE_SENSOR];
    double compass_angle;
    double actual_angle;
    double actual_angle2;
    double percentage_green;
    double percentage_yellow;
    double _left_speed, _right_speed;
    int personaencontrada;
    int line_yellow;
    int vuelta;
    int timer=0;
    int _rotate=0;
    unsigned int cuenta;


    //struct of type enum which takes all modes
    enum Mode {
        STOP,
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        FOLLOWER_MAX,
        CORNER_RIGHT,
        COMPASS,
        COMPASS2,
        LESS_LEFT,
        LESS_RIGHT,
        TURN,
        ROTATE,
        HIGHER_LEFT,
        HIGHER_RIGHT,
        BACK

    };
    //select the operating mode
    Mode _mode;

    Camera *_forward_camera;
    Camera * _spherical_camera;


public:
    /**
     * @brief Destructor of the class.
     * @param compass_angle,actual_angle,percentage_green,percentage_yellow,personaencontrada,line_yellow,left_speed,right_speed.  theres param govern we program code
     * @return compass_angle,actual_angle,percentage_green,percentage_yellow,personaencontrada,line_yellow,left_speed,right_speed
     */
    MyRobot();
    /**
     * @brief User defined function for initializing and running the template class.
     * @param distance_sensor, forward_camera, spherical_camera, compass_angle.
     * @return
     */
    ~MyRobot();
    /**
     * @brief Main program control
     * @param value of mode operation
     * @return _mode
     */
    void run();
    /**
     * @brief An example for converting bearing vector from compass to angle (in degrees).
     * @param double in_vector
     * @return value of variable convert bearing to degrees
     */
    double convert_bearing_to_degrees(const double* in_vector);
    /**
     * @brief Read all sensor to our robot
     * @param distance_sensor There value use to initialize ir_val[]:
     * @return
     */
    void read_sensor();
    /**
     * @brief Read camera spherical
     * @param we go through the pixels of the camera
     * @return percentage_yellow
     */
    int read_camera_spherical();
    /**
     * @brief function we use to dodge obstacles
     * @param ir_val[]
     * @return value of variable convert bearing to degrees
     */
    double laberinto();

    /**
     * @brief this function use to guide the robot
     * @param compass_angle
     * @return _mode
     */
    double orientar();
    /**
     * @brief this function use to guide the robot
     * @param compass_angle
     * @return _mode
     */
    double orientar2();
    /**
     * @brief Function we use to search people lost in map.
     * @param personaencontrada. when we find a people, actualize value of pernosaencontrada.
     * @return personaencontrada.
     */
    double busqueda();
    /**
     * @brief focus the percentage of green on the screen
     * @param
     * @return _mode
     */
    double orienta_green();
    /**
     * @brief we pause of 2 seconds
     * @param
     * @return
     */
    bool time_wait();
    /**
     * @brief
     * @param
     * @return
     */
    bool rotate();
    /**
     * @brief used the functions laberinto() y orineta() to move around the map.
     * @param
     * @return
     */
    void navigation();
    /**
     * @brief used the functions laberinto() y orineta() to move around the map in another sense .
     * @param
     * @return
     */
    void navigation2();
    /**
     * @brief
     * @param
     * @return
     */
    void rotate2();
};
