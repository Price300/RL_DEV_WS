#include "control_loop.h"

//We can use the class constructor to set parameters
CONTROLLER::CONTROLLER() {
    A=1;
    B=1;
    C=1;
    kp=0.001;
    ki=kp/10;
    kd=(kp*kp)/1000;
    Ts=0.1;
    first_entry=true;
    stop=false;
}


//Sense: get input to change the state of our System
void CONTROLLER::set_xdes() {
    if(!stop){
        if(first_entry){
            cout<<"Insert reference (insert 0 to stop ) ";
            cin>>reference;
            if(reference==0) stop=true;
            first_entry=false; 
        }
        else{
            cin>>reference;
            if(reference==0) stop=true;
        }       
    }
}


//Random initial value
void CONTROLLER::system_start() {
    state=rand()%100+1;
}

void CONTROLLER::loop() {
    output=state;
    float integral=0,derivative;
    float error,error_old=0;
    while(true){
        error=reference-output;
        integral+=error*Ts;
        derivative=(error-error_old)/Ts;
        input=kp*error+ki*integral+kd*derivative;
        state=A*state+B*input;
        output=C*state;
        error_old=error;
        my_file1<<output<<endl;
        my_file2<<reference<<endl;
        cout<<output<<" reference: "<<reference<<"\n";
        usleep(10000);
    }
}

void CONTROLLER::run(){ 
    system_start();
	my_file1.open("PID_output", ios::out);
    my_file2.open("PID_reference", ios::out);
	if (!my_file1 && !my_file2) {
		cout << "File not created!";
	}
	else {
		cout << "Files created successfully!\n";
	}
    while(!stop){
        boost::thread reference_thread( &CONTROLLER::set_xdes, this);
        reference_thread.join();
        boost::thread loop_thread( &CONTROLLER::loop, this);
    }
    my_file1.close(); 
    my_file2.close();
    cout<<"file closed successfully \n"; //debug
}