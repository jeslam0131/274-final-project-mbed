#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"

#define NUM_INPUTS 4
#define NUM_OUTPUTS 5

//Measured values
float velocity = 0;
float current = 0;
float theta = 0;

//Set values
float current_d = 0.5;
float kp = 0;
float ki = 0;
float kd =0;
float total_error=0;

//Controller values
float volt = 0;
float duty = 0;

Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment
Ticker ControlLoop;         // Ticker to run current controller at high frequency

QEI encoderA(PE_9,PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderC(PC_6, PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding)

MotorShield motorShield(24000); // initialize the motor shield with a period of 12000 clock ticks or ~10kHZ

// function to calculate motor voltage according to current control law
void current_control() {
    float error = 0;
    
    theta = encoderA.getPulses()*(6.2831/1200.0);
    velocity = encoderA.getVelocity()*(6.2831/1200.0);
    current = -(motorShield.readCurrentA()*(30.0/65536.0)-15.0); //read current for motor A in amps. Note: this is a slightly different current sensor so its a different conversion than last lab.            
    current_d = (-2*theta-0.01*velocity)/0.16;
    error = current_d - current;
    total_error+=error;
    if(total_error>2){
        total_error=2;
    }
    
    volt = 3.4*current_d + kp*error+ 0.16*velocity+ki*total_error; // EDIT THIS to use your current control law from Lab 2
    
    duty  = volt/12.0;
    
    if (duty >  1) {
        duty =  1;
    }
    if (duty < -1) {
        duty = -1;   
    }

    if (duty >= 0){
        motorShield.motorAWrite(duty, 0); 
    }
    else if (duty < 0){
        motorShield.motorAWrite(abs(duty), 1);
    }
}


int main (void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    
    // Define array to hold input parameters from MATLAB
    float input_params[NUM_INPUTS];
    pc.printf("%f",input_params[0]);
    
    while(1) {
        // Run experiment every time input parameters are received from MATLAB
        if (server.getParams(input_params,NUM_INPUTS)) {
            // Unpack inputs
            kp = input_params[0];
            ki = input_params[1];
            current_d = input_params[2];
            time = input_params [3];

            // Run current controller at 10kHz
            ControlLoop.attach(&current_control,0.0001);
            
            // Setup experiment
            t.reset();
            t.start();
            encoderA.reset();
            encoderB.reset();
            encoderC.reset();
            encoderD.reset();

            motorShield.motorAWrite(0, 0); //turn motor A off
            
            // Use the motor shield as follows:
            // motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION = 1 is backwards.
             
            // Run experiment
            while (t.read() < time) {
                // Perform impedance control loop logic to calculate desired current
                current_d = (-10*theta+0.001*velocity)/0.16; // Set commanded current from impedance controller here.
                
                // Send data to MATLAB
                float output_data[NUM_OUTPUTS];
                output_data[0] = t.read();
                output_data[1] = theta;
                output_data[2] = velocity;
                output_data[3] = current;
                output_data[4] = volt;

                server.sendData(output_data,NUM_OUTPUTS);              
                ThisThread::sleep_for(1); //run outer control loop at 1kHz
            }

            // Cleanup after each experiment
            ControlLoop.detach();
            server.setExperimentComplete();
            motorShield.motorAWrite(0, 0); //turn motor A off
        } // end if
    } // end while
    
} // end main