#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"

#define NUM_INPUTS 2
#define NUM_OUTPUTS 4

Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment

QEI encoderA( PE_9, PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderB( PA_5,  PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderC( PC_6,  PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING); // MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding)

MotorShield motorShield(12000); //initialize the motor shield with a period of 12000 clock ticks or ~20kHZ

int main (void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    pc.printf("%f",input_params[0]);
    
    while(1) {
        if (server.getParams(input_params, NUM_INPUTS)) {
            // Setup experiment
            t.reset();
            t.start();
            encoderA.reset();
            encoderB.reset();
            encoderC.reset();
            encoderD.reset();

            motorShield.motorAWrite(0, 0); //turn motor A off
            
            //use the motor shield as follows:
            //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
             
            // Unpack inputs
            float d1   = input_params[0]; // Duty cycle for first second
            float d2   = input_params[1]; // Duty cycle for second second

            // Run experiment
            while (t.read() < 2) {
                // Perform control loop logic
                if (t.read() < 1)
                    motorShield.motorAWrite(d1, 0); //run motor A at "v1" duty cycle and in the forward direction 
                else
                    motorShield.motorAWrite(d2, 0); //run motor A at "v2" duty cycle and in the forward direction 

                // Form output to send to MATLAB
                float output_data[NUM_OUTPUTS];
                
                output_data[0] = t.read();
                output_data[1] = 0; // MODIFY THIS: Copy in your equation for encoder angle from part 1
                output_data[2] = 0; // MODIFY THIS: Copy in your equation for encoder velocity from part 1
                output_data[3] = motorShield.readCurrentA()*(30.0/65536.0)-15;
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);
                
                ThisThread::sleep_for(1); //run control loop at 1kHz
            }
            // Cleanup after experiment
            server.setExperimentComplete();
            motorShield.motorAWrite(0, 0); //turn motor A off
        } // end if
    } // end while
} // end main

