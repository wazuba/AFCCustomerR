#ifndef STEPPER_H      
#define STEPPER_H

#include "p30fxxxx.h"
#include "p30f2023.h"
#include "libpic30.h"
#include "i2c.h"
#include "init.h"
#include "userparams.h"
#include "M24LC64F.h"
#include "MCP4725F.h"
#include <dsp.h>

    //Fault flags
    typedef 
        struct{
            unsigned ocFault			    :1;         
            unsigned cpuFault	  		    :1;         
            unsigned afcLostFault			:1;         
            unsigned warmUpFault			:1;         
            unsigned unused1		        :1;         
            unsigned unused2			    :1;         
            unsigned unused3			    :1;         
            unsigned unused4				:1;         
            unsigned               		    :8;         
    }faultFlags; 

/* Globals */
	extern int state;
	extern unsigned int IMotor1;
	extern unsigned int IMotor2;
	extern int triggerINT;
	extern int motorOrDelay;
	extern int setDir;
	extern int stepSize;
	extern unsigned int motorStep;
	extern unsigned int motorPos;
	extern unsigned int homePos; 
	extern unsigned int motorPosAddress;
	extern int errorArray[BUFFER_SIZE];
	extern int strPTR;
	extern int endPTR;
	extern int bufferFull;
	extern int error;
	extern unsigned int homeAddress;
	
	extern unsigned int sampleTrigger;
	extern faultFlags faultStatus;
	extern int sigma;
	extern int delta;
	extern tPID fooPID;
    extern fractional kCoeffs[];
    extern fractional abcCoefficient[3] __attribute__ ((section (".xbss, bss, xmemory")));
	extern fractional controlHistory[3] __attribute__ ((section (".ybss, bss, ymemory")));
	extern unsigned int count;
	extern int heatPerPulse;
	extern int prevReflectedPower;
	extern int reflectedPower;
	extern int thermalError;
	extern int coolRate;  
	extern unsigned int thermalCounter;
	extern unsigned int prevThermalCounter;
	extern int target;
    extern unsigned long heatAccumulator;
	extern int counterA;
	extern int counterB;
    
/* Functions */

void moveMotor(int direction, unsigned int speed);					//Moves the motor in a specific direction and speed
void updateAnalogOut(unsigned int data);					//Update the external DAC 
int disableINT0(void);										//Disables the external interrupt
int enableINT0(void);										//Enables the external interrupt
void checkState(void);										//Checks the state of the control signals and selects the appropriate state
void checkFaults(void);										//Checks if any fault conditions exist and reports them
void calcError(void);                                       //Calculate the error for the AFC state
void stateMachine(void);                                    //state machine that controls the software operation
void indexMotor(int returnPosition, int stepsToStop);
int calcThermalError(void);
void moveMotorThermal();
#endif
