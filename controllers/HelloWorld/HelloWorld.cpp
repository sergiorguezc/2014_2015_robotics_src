/**
 * @file    HelloWorld.cpp
 * @brief   Hello world example in webots.
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date    2014-07
 */

#include <iostream>
#include <webots/Robot.hpp>

using namespace std;
using namespace webots;

#define TIME_STEP 32

class HelloWorld : public Robot {
    public:
        HelloWorld() : Robot() {
        }

        virtual ~HelloWorld() {
        }

        void run() {
            int i = 0;
        
            // main control loop
            while (step(TIME_STEP) != -1) {
                cout << "Hello world: " << i << endl;
                i += 1;
            }
        }
};

int main(int argc, char **argv) {
    HelloWorld *hw = new HelloWorld();
    hw->run();
    delete hw;

    return 0;
}
