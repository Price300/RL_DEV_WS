#include <iostream>
#include "boost/thread.hpp"
#include <fstream>
#include <unistd.h>

using namespace std;

class CONTROLLER {
    public:
        CONTROLLER();

        float reference;
        float output;
        fstream my_file1,my_file2;
        
        void loop();                //Main loop function        
        void system_start();       //start the system
        void set_xdes();   //member to set the desired value
        void run();

    private:
        float state;
        float input;
        float A,B,C;
        float kp,ki,kd;
        float Ts;
        bool first_entry;
        bool stop;
};