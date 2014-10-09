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
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _mode = FORWARD;

    _distance_sensor[0] = getDistanceSensor("ds1");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds13");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds14");
    _distance_sensor[2]->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double ir1_val = 0.0, ir13_val = 0.0, ir14_val = 0.0;

    while (step(_time_step) != -1) {
        // Read the sensors
        ir1_val = _distance_sensor[0]->getValue();
        ir13_val = _distance_sensor[1]->getValue();
        ir14_val = _distance_sensor[2]->getValue();

        // Control logic of the robot
        if (_mode == FORWARD) {
            // Move forward

            // When sufficiently close to a wall in front of robot,
            // switch mode to wall following
            if ((ir1_val > DISTANCE_LIMIT) || (ir14_val > DISTANCE_LIMIT)) {
                _mode = WALL_FOLLOWER;
                cout << "Mode " << WALL_FOLLOWER << ": Wall following mode activated" << endl;
            }
        }
        else {
            // Wall following

            if ((ir1_val > DISTANCE_LIMIT) || (ir14_val > DISTANCE_LIMIT)) {
                _mode = WALL_FOLLOWER;
                cout << "Backing up and turning left." << endl;
            }
            else {
                if (ir13_val > DISTANCE_LIMIT) {
                    _mode = TURN_LEFT;
                    cout << "Turning left." << endl;
                }
                else {
                    if (ir13_val < DISTANCE_LIMIT + 50) {
                        _mode = TURN_RIGHT;
                        cout << "Turning right." << endl;
                    }
                    else {
                        _mode = FORWARD;
                        cout << "Moving forward." << endl;
                    }
                }
            }
        }

        // Send actuators commands according to the mode
        switch (_mode){
            case STOP:
                _left_speed = 0;
                _right_speed = 0;
                break;
            case FORWARD:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
                break;
            case TURN_LEFT:
                _left_speed = MAX_SPEED / 1.25;
                _right_speed = MAX_SPEED;
                break;
            case TURN_RIGHT:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED / 1.25;
                break;
            case WALL_FOLLOWER:
                _left_speed = -MAX_SPEED / 3.0;
                _right_speed = -MAX_SPEED / 20.0;
                break;
            default:
                break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////
