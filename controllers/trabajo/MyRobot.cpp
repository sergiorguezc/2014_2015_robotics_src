/**
 * @file    MyRobot.cpp
 * @brief   trabajo.
 *
 * @author  Carlos Hernandez Paz <100292793@alumnos.uc3m.es>, Sergio Rodríguez Coronel <100081046@alumnos.uc3m.es>
 * @date    02-12-2014
 */

#include "MyRobot.h"
//////////////////////////////////////////////
MyRobot::MyRobot() : DifferentialWheels()
{

    _time_step = 64;
    percentage_green = 0.0;
    percentage_yellow = 0.0;
    personaencontrada = 0;
    line_yellow = 0;
    vuelta = 0;
    actual_angle = 80;
    _left_speed = 0;
    _right_speed = 0;


    for(int i=0;i<16;i++){
        string Result;
        ostringstream convert;   // stream used for the conversion
        convert << i;      // insert the textual representation of 'Number' in the characters in the stream
        Result = convert.str();
        _distance_sensor[i]=getDistanceSensor("ds"+ Result);
        _distance_sensor[i]->enable(_time_step);
    }
    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);
}
///////////////////////////////////////////////////////
MyRobot::~MyRobot()
{
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
    _my_compass->disable();
    _forward_camera->disable();
    _spherical_camera->disable();
}
///////////////////////////////////////////////////////
void MyRobot::run()
{

    int sum = 0;
    unsigned char green = 0, red = 0, blue = 0;


    // get size of images for forward camera
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = _forward_camera->getHeight();

    for(int i=0;i<NUM_DISTANCE_SENSOR;i++){
        ir_val[i]= 0.0;
    }




    //loop who finsih when the time step get the value of -1
    while (step(_time_step) != -1) {

        /*********************SENSOR****************************/
        // Read the sensors
        read_sensor();


        /*********************COMPASS**********************************/
        // Read the compass
        const double *compass_val = _my_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);


        /***********************CAMERA************************************/
        sum = 0;

        // get current image from forward camera
        const unsigned char *image_f = _forward_camera->getImage();

        // count number of pixels that are white
        // (here assumed to have pixel value > 245 out of 255 for all color components)

        for (int x = 0; x < image_width_f; x++) {
            for (int y = 0; y < image_height_f; y++) {
                green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
                red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
                blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);
                if (((green > 100) && (red < 90) && (blue < 90))) {
                    sum = sum + 1;
                }
            }
        }
        percentage_green = (sum / (float) (image_width_f * image_height_f)) * 100;
        //cout << "Percentage of green in forward camera image: " << percentage_green << endl;

        /***************************************LOGIG CONTROL********************************************/
        // This is the logic control permiete us navigate around the map and find people
        //condition to activate navigation() function
        if((percentage_green < 10) && (personaencontrada == 0)){
            navigation();
        }
        else{
            //condition to activate busqueda() function
            if((percentage_green > 10)&&(personaencontrada == 0)){
                busqueda();
            }
            else{
                //condition to return to top
                if(personaencontrada == 1 ){
                    actual_angle2 = -105;
                    read_camera_spherical();
                    navigation2();
                }
                else{
                    //condition to stop the program
                    if((cuenta > 2)&&(personaencontrada==1)&&((ir_val[0]==0)&&(ir_val[1]==0)&&(ir_val[2]==0)&&(ir_val[3]==0)&&(ir_val[4]==0)&&(ir_val[5]==0)&&(ir_val[6]==0)&&(ir_val[7]==0)&&(ir_val[8]==0)&&(ir_val[9]==0)&&(ir_val[10]==0)&&(ir_val[11]==0)&&(ir_val[12]==0)&&(ir_val[13]==0)&&(ir_val[14]==0)&&(ir_val[15]==0)) ){
                        _mode = STOP;
                    }

                }
            }
        }

        switch (_mode){
            //robot moves to the left abruptly
        case HIGHER_LEFT:
            _left_speed = 10;
            _right_speed = 60;
            break;
            //robot moves to the right abruptly
        case HIGHER_RIGHT:
            _left_speed = 60;
            _right_speed = 5;
            break;

            //Stop the speed of the robot sending two values of zero to each wheel
        case STOP:

            _left_speed = 0;
            _right_speed = 0;
            break;

            //The robot move forward sending two values of max speed to each wheel
        case FORWARD:

            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED;
            break;

            //The right wheel have a higher speed than the left one, to turn in the direction
        case TURN_LEFT:

            _left_speed = 35;
            _right_speed = 60;
            break;

            //The left wheel have a higher speed than the right one, to turn in the direction
        case TURN_RIGHT:

            _left_speed = 60;
            _right_speed = 15;
            break;

            //this mode is use to turn faster, with a higher angle
        case FOLLOWER_MAX:
            _left_speed=-60;
            _right_speed=0;
            break;

            //this mode is use to turn right faster, with a higher angle
        case CORNER_RIGHT:
            _left_speed=0;
            _right_speed= -60;
            break;

            //this mode turn right the robot progressive
        case LESS_RIGHT:
            _left_speed=60;
            _right_speed=55;
            break;

            //this mode turn left the robot progressive
        case LESS_LEFT:
            _left_speed=55;
            _right_speed=60;
            break;

            //this mode help us to reach the end
        case TURN:
            _left_speed=60;
            _right_speed=15;
            break;

            //this mode is use to rotate robot

        case ROTATE:
            _left_speed = -20;
            _right_speed = 20;
            break;

            //This mode the robot go back
        case BACK:
            _left_speed = -20;
            _right_speed = -20;
            break;

            //this mode help us to stright the robot with a 55 angle
        case COMPASS:
            if (compass_angle < (actual_angle - 2)) {
                _left_speed = 90;
                _right_speed = 30;

            }
            else {
                if (compass_angle > (actual_angle + 2)) {

                    _left_speed = 30;
                    _right_speed = 90;

                }
                else {

                    _left_speed = 70;
                    _right_speed = 70;
                }
            }
            break;

             //this mode help us to stright the robot with a -105 angle
        case COMPASS2:
            if (compass_angle < (actual_angle2 - 2)) {
                // Turn right
                _left_speed = 90;
                _right_speed = 30;

            }
            else {
                if (compass_angle > (actual_angle2 + 2)) {
                    // Turn left
                    _left_speed = 30;
                    _right_speed = 90;

                }
                else {
                    // Move straight forward
                    _left_speed = 80;
                    _right_speed = 80;
                }
            }

            break;

        default:
            break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);

    }
}
///////////////////////////////////////////////////////

//becomes the angle in degrees
double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}
////////////////////////READ SENSOR////////////////////
void MyRobot::read_sensor(){

    for(int i=0;i<NUM_DISTANCE_SENSOR;i++){
        ir_val[i]=_distance_sensor[i]->getValue();

    }
}
////////////////////////READ CAMERA SPHERICAL//////////
int MyRobot::read_camera_spherical(){

    int sum_a = 0;
    unsigned char green = 0, red = 0, blue = 0;


    // Get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();


    // Get current image from spherical camera
    const unsigned char *image_s = _spherical_camera->getImage();

    // for detected yellow: green > 204, red > 202, blue < 60
    for (int x = 0; x < image_width_s; x++) {
        for (int y = 0; y < image_height_s; y++) {
            green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
            red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
            blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);
            //value RGB to detected yellow color
            if(((green > 204) && (red > 204) && (blue < 60))){
                sum_a = sum_a + 1;
            }
        }
    }


    percentage_yellow = (sum_a / (float) (image_width_s * image_height_s)) * 100;
    //cout << "Percentage of yellow in spherical camera image: " << percentage_yellow << endl;

    // use to detected line_yellow
    if((percentage_yellow > 1.45)){
        cuenta++;
    }

}
////////////////////////NAVIGATION/////////////////////
void MyRobot::navigation(){
    //condition to activate the function laberinto()
    if(((ir_val[0]>30)||(ir_val[1]>30)||(ir_val[2]>30)||(ir_val[3]>30)||(ir_val[4]>30)||(ir_val[5]>30)||(ir_val[6]>30)||(ir_val[7]>30)||(ir_val[8]>30)||(ir_val[9]>30)||(ir_val[10]>30)||(ir_val[11]>30)||(ir_val[12]>30)||(ir_val[13]>30)||(ir_val[14]>30)||(ir_val[15]>30))){
        laberinto();
    }
    else{
        //condition to activate the function orientar().
        if((ir_val[0]<30)||(ir_val[1]<30)||(ir_val[2]<30)||(ir_val[3]<30)||(ir_val[4]<30)||(ir_val[5]<30)||(ir_val[6]<30)||(ir_val[7]<30)||(ir_val[8]<30)||(ir_val[9]<30)||(ir_val[10]<30)||(ir_val[11]<30)||(ir_val[12]<30)||(ir_val[13]<30)||(ir_val[14]<30)||(ir_val[15]<30)){
            orientar();
        }
    }
}
////////////////////////NAVIGATION2////////////////////

//this function is equal to previous navigation with the diference, in this function we use orientar2() back to initiation
void MyRobot::navigation2(){
    if(((ir_val[0]>30)||(ir_val[1]>30)||(ir_val[2]>30)||(ir_val[3]>30)||(ir_val[4]>30)||(ir_val[5]>30)||(ir_val[6]>30)||(ir_val[7]>30)||(ir_val[8]>30)||(ir_val[9]>30)||(ir_val[10]>30)||(ir_val[11]>30)||(ir_val[12]>30)||(ir_val[13]>30)||(ir_val[14]>30)||(ir_val[15]>30))){
        laberinto();

    }
    else{
        if((ir_val[0]<30)||(ir_val[1]<30)||(ir_val[2]<30)||(ir_val[3]<30)||(ir_val[4]<30)||(ir_val[5]<30)||(ir_val[6]<30)||(ir_val[7]<30)||(ir_val[8]<30)||(ir_val[9]<30)||(ir_val[10]<30)||(ir_val[11]<30)||(ir_val[12]<30)||(ir_val[13]<30)||(ir_val[14]<30)||(ir_val[15]<30)){
            orientar2();

        }
    }
}
////////////////////////LABERINTO//////////////////////
double MyRobot::laberinto(){


    // Control logic of the robot
    //este if se encarga de conocer si hay algun sensor activo, en caso de que el robot no este,
    //proximo a ningun objeto, seguira el camino atendiendo a la brujula, de lo contrario entraría dentro del if
    if((ir_val[0]>40)||(ir_val[1]>40)||(ir_val[2]>40)||(ir_val[3]>40)||(ir_val[4]>40)||(ir_val[5]>40)||(ir_val[6]>40)||(ir_val[7]>40)||(ir_val[8]>40)||(ir_val[9]>40)||(ir_val[10]>40)||(ir_val[11]>40)||(ir_val[12]>40)||(ir_val[13]>40)||(ir_val[14]>40)||(ir_val[15]>40)){

        //En este caso si los sensores delantero-laterales estan activos
        //el robot retrocederá para evitar colision y mientras se orientará a la izquierda

        if(((ir_val[15]>DISTANCE_LIMIT-60)||(ir_val[0]>DISTANCE_LIMIT-60))&&(ir_val[12]>DISTANCE_LIMIT-40)&&(ir_val[3]<DISTANCE_LIMIT+50)){
            _mode=FOLLOWER_MAX;}
        else{
            // if sensor 15, 0, 3 detected wall and sensor 12 is void, the robot the robot fast turn right
            if(((ir_val[15]>DISTANCE_LIMIT-60)||(ir_val[0]>DISTANCE_LIMIT-60))&&(ir_val[3]>DISTANCE_LIMIT-40)&&(ir_val[12]<DISTANCE_LIMIT+50)){
                _mode=CORNER_RIGHT;}
            else{
                //if sensor 2, 1, 14 and 13 are activated, we are in a corner
                if(((ir_val[2]>DISTANCE_LIMIT-60)&&(ir_val[13]>DISTANCE_LIMIT-60)&&(ir_val[14]>DISTANCE_LIMIT-60)&&(ir_val[1]>DISTANCE_LIMIT-60))){
                    _mode=ROTATE;}
                else{
                    // if sensor 15 or 14 or 13 activated we Turn left abruptly
                    if((ir_val[15]>DISTANCE_LIMIT-60)&&(ir_val[14]>DISTANCE_LIMIT-70)&&(ir_val[13]>DISTANCE_LIMIT-80)){
                        _mode=HIGHER_LEFT;}
                    else{
                        // if sensor 1 or 2 or 0 activated we Turn right abruptly
                        if((ir_val[1]>DISTANCE_LIMIT-70)&&(ir_val[2]>DISTANCE_LIMIT-80)&&(ir_val[0]>DISTANCE_LIMIT-60)){
                            _mode=HIGHER_RIGHT;}
                        else{
                            //if sensor 11 or 12 activated we turn left to wall separating
                            if((ir_val[11]>DISTANCE_LIMIT-50)&&(ir_val[12]<DISTANCE_LIMIT-65)){
                                _mode=TURN;}
                            else{
                                // if sensor 4,5,3 activated we turn right to followin the wall
                                if((((ir_val[4]>DISTANCE_LIMIT-40)&&(ir_val[3]>DISTANCE_LIMIT-30))||(ir_val[2]>DISTANCE_LIMIT-30))&&(ir_val[15]<DISTANCE_LIMIT)){
                                    _mode=TURN_RIGHT;}
                                else{
                                    // if sensor 11,12,33 activated we turn left to followin the wall
                                    if((((ir_val[11]>DISTANCE_LIMIT-40)&&(ir_val[12]>DISTANCE_LIMIT-30))||(ir_val[13]>DISTANCE_LIMIT-30))&&(ir_val[0]<DISTANCE_LIMIT)){
                                        _mode=TURN_LEFT;}
                                    else{
                                        // if sensor 4,5 activated we turn left to guide wall
                                        if((ir_val[4]>DISTANCE_LIMIT-60)&&(ir_val[5]>DISTANCE_LIMIT-60)){
                                            _mode=LESS_LEFT;}
                                        else{
                                            // if sensor 10,9 activated we turn right to guide wall
                                            if((ir_val[10]>DISTANCE_LIMIT-60)&&(ir_val[9]>DISTANCE_LIMIT)){
                                                _mode= HIGHER_RIGHT;}
                                            else{
                                                //if sensor 14 activated we Turn right abruptly
                                                if((ir_val[14]>DISTANCE_LIMIT-60)&&(ir_val[1]<DISTANCE_LIMIT)){
                                                    _mode=HIGHER_RIGHT;
                                                }
                                                else{
                                                    // if sensor 11,10 activated we turn right to guide wall
                                                    if((ir_val[11]>DISTANCE_LIMIT-60)&&(ir_val[10]>DISTANCE_LIMIT-60)){
                                                        _mode=HIGHER_RIGHT;}
                                                    else{
                                                        // if sensor 11,12,4,3 activated we are in a corridor and turn right because we prefer right wall
                                                        if((ir_val[11]>DISTANCE_LIMIT-60)&&(ir_val[12]>DISTANCE_LIMIT-40)&&(ir_val[3]>DISTANCE_LIMIT)&&(ir_val[4]>DISTANCE_LIMIT)){
                                                            _mode=TURN_RIGHT;
                                                        }
                                                        else{
                                                            //if sensor 8, 9 activated we go forward
                                                            if((ir_val[8]>DISTANCE_LIMIT)&&(ir_val[9]>DISTANCE_LIMIT)){

                                                                _mode=FORWARD;}
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else{
        _mode = FORWARD;
    }

    return _mode;

}
////////////////////////ORIENTAR///////////////////////

// This function is used to compare the angle of navigation with the current robot
double MyRobot::orientar(){

    if((compass_angle != actual_angle)){
        _mode = COMPASS;
    }

    return _mode;
}
////////////////////////ORIENTA2///////////////////////
double MyRobot::orientar2(){

    if((compass_angle != actual_angle)){
        _mode = COMPASS2;

    }

    return _mode;
}
////////////////////////BUSQUEDA///////////////////////
//This function serves to find people by green
double MyRobot::busqueda(){


    if((percentage_green == 0)&& (vuelta == 0)){
        _mode = ROTATE;

    }
    else{

        if(percentage_green != 0){
            orienta_green();

            if((ir_val[0]>DISTANCE_LIMIT -60)||(ir_val[15]>DISTANCE_LIMIT -60)){
                _mode=STOP;

                int wait=time_wait();
                if(wait==false){
                    time_wait();
                    _mode=STOP;
                }
                else{
                    personaencontrada++;
                }
            }

        }

    }
}
////////////////////////Orienta_green//////////////////
//this function serves two mantain the green in the center of the screen
// we use a mask image
double MyRobot:: orienta_green(){

    int mask_width_f = _forward_camera->getWidth();
    int mask_height_f = _forward_camera->getHeight();
    double percentage_green_left;
    double percentage_green_right;
    unsigned char green = 0, red = 0, blue = 0;
    const unsigned char *image_f = _forward_camera->getImage();
    int sum_i=0;
    int sum_d=0;
    // This part covers the right side of the screen and gives us a percentage of green
    for (int x = 0; x < mask_width_f/2; x++) {
        for (int y = 0; y < mask_height_f; y++) {
            green = _forward_camera->imageGetGreen(image_f, mask_width_f, x, y);
            red = _forward_camera->imageGetRed(image_f, mask_width_f, x, y);
            blue = _forward_camera->imageGetBlue(image_f, mask_width_f, x, y);
            //value RGB to detected color green
            if (((green > 100) && (red < 90) && (blue < 90))) {

                sum_i = sum_i + 1;
            }
        }
    }
    percentage_green_left = (sum_i / (float) (mask_width_f * mask_height_f)) * 100;
    //This part covers the left side of the screen and gives us a percentage of green
    for (int x = mask_width_f/2; x < mask_width_f; x++) {
        for (int y = 0; y < mask_height_f; y++) {
            green = _forward_camera->imageGetGreen(image_f, mask_width_f, x, y);
            red = _forward_camera->imageGetRed(image_f, mask_width_f, x, y);
            blue = _forward_camera->imageGetBlue(image_f, mask_width_f, x, y);
            if (((green > 100) && (red < 90) && (blue < 90))) {

                sum_d = sum_d + 1;
            }
        }
    }
    percentage_green_right = (sum_d / (float) (mask_width_f * mask_height_f)) * 100;
    //logic control to guide the robot
    if((percentage_green_left > percentage_green_right)){
        _mode = TURN_LEFT;}
    else{
        if((percentage_green_left < percentage_green_right)){
            _mode = TURN_RIGHT;}
        else{
            _mode = FORWARD;}
    }

    return _mode;
}
////////////////////////Time_wait//////////////////////
// function to stop time 2 seconds
bool MyRobot::time_wait(){
    if(timer<70){
        _mode = STOP;
        timer++;
        return false;
    }
    else{
        return true;
    }

}
////////////////////////Rotate/////////////////////////
//Test function to rotate the robot on itself
bool MyRobot::rotate(){
    if(_rotate<200){
        _mode=ROTATE;
        _rotate++;
        return false;
    }
    else{
        return true;
    }
}
////////////////////////Rotate2////////////////////////
//Test function to rotate the robot on itself
void MyRobot::rotate2(){
    _mode = ROTATE;
    if(ir_val[7]>30){
        _mode = STOP;
        vuelta++;
    }
}
