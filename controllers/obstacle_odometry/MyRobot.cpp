/**
 * @file    MyRobot.cpp
 * @brief  obstacle_odometry.
 *
 * @author  Sergio Rodríguez Coronel <10081046@alumnos.uc3m.es>
 * @date    2014-07
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    // Initialize variable

    _left_speed = 0;
    _right_speed = 0;
    _posicion_final = 0.0;
    _encoder_right = 0.0;
    _encoder_left = 0.0;
    _Radio = 0.0825;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    //Initialize mode.
    _mode = FORWARD;

    enableEncoders(_time_step);

}


//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _my_compass->disable();
    disableEncoders();

}

//////////////////////////////////////////////

void MyRobot::run()
{
    double compass_angle;
    double _angle_compare;

    while (step(_time_step) != -1) {

        // Read the sensors
        const double *compass_val = _my_compass->getValues();
        // get value encoders
        _encoder_left = getLeftEncoder();
        _encoder_right = getRightEncoder();



        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);
        _angle_compare = -43;

        // Print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;

        _posicion_final = _encoder_left/1000 *0.0825;
        cout << "posicion :" << _posicion_final << endl;

        // estrategia de control
        // The strategy followed is to calculate the position of the robot by the encoder
        // function of the conditions set execute different behaviors.

        // first control condition
        if (_posicion_final < 0.015){

            // with this structure control errors by the value of the encoder
            if ((_encoder_right) > (_encoder_left)) {
                // Turn right
                _mode = TURN_RIGHT;
                cout << " encoder_right : " << _encoder_right << " encoder_left : " << _encoder_left << endl;
                cout << " Turn Right " << endl;
            }
            else {
                if ((_encoder_right) < (_encoder_left)) {
                    // Turn left
                    _mode = TURN_LEFT;
                    cout << " encoder_right : " << _encoder_right << " encoder_left : " << _encoder_left << endl;
                    cout << " Turn Left1 " << endl;
                }
                else {
                    // Move straight forward
                    _mode = FORWARD;
                }
            }
        }
        else{
            // second control condition
            if(_posicion_final < 0.028 && compass_angle > -43){

                _mode = TURN_LEFT;// turn left
                cout << " Turn Left2 " << endl;
            }
            else{
                // third control condition
                if( compass_angle < -43 && _posicion_final < 0.028){
                    _mode = FORWARD;
                    cout << " Forward " << endl;

                }

                else{
                    // quarter control condition
                    if(_posicion_final > 0.025 && compass_angle < 45){
                        _mode = TURN_RIGHT;
                        cout << " Turn Right " << endl;

                    }
                    else{
                        // with this structure control errors by the value of angle.
                        if(compass_angle > 45 ){
                            /* _mode = FORWARD;
                            cout << " Forward2 " << endl;*/

                            if (compass_angle < (DESIRED_ANGLE - 2) && _posicion_final >0.1) {
                                // Turn right
                                _mode = TURN_RIGHT;
                                cout << " encoder_right : " << _encoder_right << " encoder_left : " << _encoder_left << endl;
                                cout << " Turn Right " << endl;
                            }
                            else {
                                if (compass_angle > (DESIRED_ANGLE + 2) && _posicion_final >0.1) {
                                    // Turn left
                                    _mode = TURN_LEFT;
                                    cout << " encoder_right : " << _encoder_right << " encoder_left : " << _encoder_left << endl;
                                    cout << " Turn Left1 " << endl;
                                }
                                else {
                                    // Move straight forward
                                    _mode = FORWARD;
                                }
                            }
                        }
                        // fifth control condition
                        else if(_posicion_final > 0.1){
                            _mode = STOP;
                            cout << " STOP "<< endl;
                        }
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
            _left_speed = MAX_SPEED ;
            _right_speed = MAX_SPEED;
            break;
        case TURN_LEFT:
            _left_speed = MAX_SPEED /1.25 ;
            _right_speed = MAX_SPEED ;
            break;
        case TURN_RIGHT:
            _left_speed = MAX_SPEED ;
            _right_speed = MAX_SPEED /1.25;
            break;

        default:
            break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////
