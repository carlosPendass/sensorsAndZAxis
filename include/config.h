//The line below makes the compiler to copy the config.h once, even is it's called multiples times. 
// #pragma one
/////////////////////////////////////////////////////////////
#define timeBsensors 50
#define CONSTOFREJECTION 1000
#define SIMPLE 0
#define FULL 1
#define GROUPOFSENSORS 2
#define REJECTION 3
 //Pins below ares used to select the 8bits muxes 
 #define mux8_0 5
 #define mux8_1 17
 #define mux8_2 16
 //Pins for the 4 selectors of the 16bits muxes. 
 #define mux16_0 4
 #define mux16_1 2
 #define mux16_2 15
 #define mux16_3 12
//Data IN pins from 16 bits muxes
 #define mux16Out_0 21
 #define mux16Out_1 35
 #define mux16Out_2 34 
 #define mux16Out_3 19
 #define mux16Out_4 18

/////////////////////////////////////////////////////////////
#define I2C_SLAVE_ADDR 0x04
#define SDA_PIN 22
#define SCL_PIN 23

#define AMOUNTOFSQUARES 100
#define AMOUNTOFSENSORS 5
#define MessageSize 32

/////////////////////////////////////////////////////////////
// Pinout for elevator

// TB6612FNG Pins
#define control1 14 //AIN1
#define control2 25 //AIN2
#define PWM 26 //PWMA

// Econder Pins
#define encoderA 32
#define encoderB 33
//Pins and constants for PWMN
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 490
#define PWM_RESOLUTION 8
//Constants for PID
#define PROPORTIONAL_CONSTANT 1.5
#define INTEGRAL_CONSTANT 0.2 
#define DERIVATIVE_CONSTANT 0.1
#define PID_OUTPUTLIMITS 200 //200 Those are the limits that our PWM signal can have
#define TIME_PID 130         // This is our sampling time, it was calculate based on the following: encoder can give one pulse every 0.26ms, so we toke a half. (uSeconds)
#define PID_TOL  10           // this is the Tolerance accepted once 20 ms have past.    
#define TIME_PIDTol 20       // This is the time we give our PID to reach a Steady State and asking for the result. 
//mechanic limits
#define MEC_TOL 100
//definitions for calibration
#define TIME_LIMITS 25 // time it waits fot the reading on calibration to be steady. 
#define PWM_LIMITS 80