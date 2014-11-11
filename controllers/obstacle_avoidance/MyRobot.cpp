/**
 * @file    obstacle_avoidance.cpp
 *
 * @author  Sergio Rodríguez Coronel <100081046@alumnos.uc3m.es>
 * @date    2014-10
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _mode = FORWARD;


    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds2");
    _distance_sensor[2]->enable(_time_step);
    _distance_sensor[3] = getDistanceSensor("ds3");
    _distance_sensor[3]->enable(_time_step);
    _distance_sensor[4] = getDistanceSensor("ds4");
    _distance_sensor[4]->enable(_time_step);
    _distance_sensor[5] = getDistanceSensor("ds5");
    _distance_sensor[5]->enable(_time_step);
    _distance_sensor[6] = getDistanceSensor("ds6");
    _distance_sensor[6]->enable(_time_step);
    _distance_sensor[7] = getDistanceSensor("ds7");
    _distance_sensor[7]->enable(_time_step);
    _distance_sensor[8] = getDistanceSensor("ds8");
    _distance_sensor[8]->enable(_time_step);
    _distance_sensor[9] = getDistanceSensor("ds9");
    _distance_sensor[9]->enable(_time_step);
    _distance_sensor[10] = getDistanceSensor("ds10");
    _distance_sensor[10]->enable(_time_step);
    _distance_sensor[11] = getDistanceSensor("ds11");
    _distance_sensor[11]->enable(_time_step);
    _distance_sensor[12] = getDistanceSensor("ds12");
    _distance_sensor[12]->enable(_time_step);
    _distance_sensor[13] = getDistanceSensor("ds13");
    _distance_sensor[13]->enable(_time_step);
    _distance_sensor[14] = getDistanceSensor("ds14");
    _distance_sensor[14]->enable(_time_step);
    _distance_sensor[15] = getDistanceSensor("ds15");
    _distance_sensor[15]->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
    _my_compass->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{

    double compass_angle;

    while (step(_time_step) != -1) {
        // Read the sensors
        ReadSensor();
        const double *compass_val = _my_compass->getValues();
        compass_angle = convert_bearing_to_degrees(compass_val);



        // Control logic of the robot

        if ((ir_val[0] > DISTANCE_LIMIT) || (ir_val[1] > DISTANCE_LIMIT) || (ir_val[14] > DISTANCE_LIMIT) || (ir_val[15] > DISTANCE_LIMIT)) {
            _mode = OBSTACLE_AVOID_R;
            cout << "Backing up and turning right." << endl;

            // condition for the robot avoid obstacle.
        }
        else {
            if(( ir_val[4] > DISTANCE_LIMIT + 20)){
                _mode = TURN_RIGHT;
                cout << " turn right "<< ir_val[4] << endl;

            }
            else if(( ir_val[4] > DISTANCE_LIMIT - 20)){
                _mode = TURN_LEFT;
                cout << " turn left " << ir_val[4] << endl;
            }

            // condition for the robot follow the wall.

            else if((ir_val[3] > ir_val[4])){
                _mode = TURN_LEFT;
                cout << "turn corner " << endl;

                // condition for the robot turn corner.
            }

            /* else{
                do{
                    _mode = WALL_FOLLOWER;
                    cout << " wall follower " << endl;
                }while( ir_val[4] > DISTANCE_LIMIT);

            }*/
            _mode = FORWARD;
            cout << "forward " << endl;

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
            _right_speed = 0;
            break;
        case TURN_RIGHT:
            _left_speed = 0;
            _right_speed = MAX_SPEED / 1.25;
            break;
        case OBSTACLE_AVOID_L:
            _left_speed = -MAX_SPEED / 3.0;
            _right_speed = -MAX_SPEED / 20.0;
            break;
        case OBSTACLE_AVOID_R:
            _left_speed = -MAX_SPEED / 20.0;
            _right_speed = -MAX_SPEED / 3.0;
            break;
        case WALL_FOLLOWER:
            _left_speed = 0.0;
            _right_speed = MAX_SPEED;
            break;
        default:
            break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}


//////////////////////////////////////////////

/* with this function compute the angle */
double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

/* with this function we value the distance sensors */
void MyRobot ::ReadSensor()
{
    for(int i = 0 ; i < NUM_DISTANCE_SENSOR; i++){
        ir_val[i] = _distance_sensor[i]->getValue();

    }
}
