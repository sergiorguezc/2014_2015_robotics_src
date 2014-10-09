/**
 * @file    main_template.cpp
 * @brief   A template for webots projects.
 *
 * @author  Name Surname <nick@alumnos.uc3m.es>
 * @date    2014-07
 */

// You may need to add general include files such as
// <iostream>, <cmath>, etc.


// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
#include <webots/DifferentialWheels.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// You may need to define some macros, only if it is necessary, like
// #define MAX_SPEED 100, #define DESIRED_ANGLE 0.0


// Here is the declaration class of your controller.
// This class declares how to initialize and how to run your controller.
// Note that this class derives from DifferentialWheels who derives from Robot,
// both inherits all its functions
class MyRobot : public DifferentialWheels {
    private:
        // You may need to define your private methods or variables, like
        //  Compass* _my_compass;
        int _time_step;

    public:
        // You may need to define your private methods or variables, like
        //  Constructors, helper functions, etc.
    
        /**
         * @brief Empty constructor of the class.
         */
        MyRobot();
        
        /**
         * @brief Destructor of the class.
         */
        ~MyRobot();

        /**
         * @brief User defined function for initializing and running the template class.
         */
        void run();
};
