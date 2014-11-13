/**
 * @file    MyRobot.cpp
 * @brief   lines_detector.
 *
 * @author  Sergio Rodríguez Coronel <100081046@alumnos.uc3m.es>
 * @date    2014-11
 */
#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;


    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);

    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[14] = getDistanceSensor("ds14");
    _distance_sensor[14]->enable(_time_step);
    percentage_yellow = 0.0;

    _mode = FORWARD;
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _spherical_camera->disable();
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
}

//////////////////////////////////////////////

void MyRobot::run()
{

    unsigned char green = 0, red = 0, blue = 0;




    // Get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();
    cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;



    while (step(_time_step) != -1) {
        int sum = 0;
        ir_val[1] = _distance_sensor[1]->getValue();
        ir_val[14] = _distance_sensor[14]->getValue();

        // ReadSensor();

        // logic Control
        if((ir_val[1] < DISTANCE_LIMIT) && (ir_val[14] < DISTANCE_LIMIT)){
            _mode = FORWARD;
            cout << " Forward " << endl;

            // Get current image from forward camera
            const unsigned char *image_s = _spherical_camera->getImage();

            // Count number of pixels that are white
            // for detected yellow: green > 245, red > 245, blue < 30
            for (int x = 0; x < image_width_s; x++) {
                for (int y = 0; y < image_height_s; y++) {
                    green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
                    red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
                    blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);


                    if ((green > 245) && (red > 245) && (blue < 30)) {
                        percentage_yellow = (sum / (float) (image_width_s * image_height_s)) * 100;

                        if(percentage_yellow > 0.6){
                            cout << " detected Yellow " << percentage_yellow  << endl;
                        }
                        else{
                            sum++;
                        }

                    }
                }
            }


        }
        // condition to avoid obstacle
        else{
            if(ir_val[14] > DISTANCE_LIMIT){
                _mode = TURN_LEFT;
                cout << " Turn LEFT " << endl;
            }
            else{
                if(ir_val[1] > DISTANCE_LIMIT){
                    _mode = TURN_RIGHT;
                    cout << "Turn RIGHT " << endl;
                }
                else{
                    _mode = STOP;
                    cout << " STOP " <<endl;
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
            _left_speed = -MAX_SPEED / 3.0;
            _right_speed = -MAX_SPEED / 20.0;
            break;
        case TURN_RIGHT:
            _left_speed = -MAX_SPEED / 20.0;
            _right_speed = -MAX_SPEED / 3.0;
            break;
        default:
            break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

/*void MyRobot ::ReadSensor()
{
    for(int i = 0 ; i < NUM_DISTANCE_SENSOR; i++){
        ir_val[i] = _distance_sensor[i]->getValue();

    }
}*/

