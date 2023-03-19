#include "control_loop.h"

using namespace std;

//the goal is to implement a simple PID controller, 
//running on input value to reach the desired one

// Sense: read a value from keyboard
// Plan:  generate the correct input
// Act:   set the input

int main(int argc, char** argv) {

    CONTROLLER controller;

    controller.run();
    cout<<"system stopped by user \n\n";

    return 0;
}
