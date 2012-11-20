#ifndef USERPARAMS_H       
#define USERPARAMS_H

/********************** dsPIC parameters *******************************/

	#define FCY 29100000 										/* FRC = 14.55MHz , PLL= X32 , FCY = 14.55e6*32/8/2 = 29.1e6 */ 
	#define FPWM 40000  										/* desired PWM frquency 40e3 */
	#define I2C_CLK                   	100000					/* I2C running at 100KHz */
	#define PGD_CONST				  	0.000000250
	#define I2C_BAUD_RATE_GENERATOR		0x11B					/*(unsigned int)((1/I2C_CLK - PGD_CONST)*FCY - 1)*/

	#define DEADTIME (unsigned int)(0.000005*FCY)
	#define	TIMER_PERIOD  	0x0256         
	#define	TIMER_PERIOD2 	0x0FFF
	#define	TIMER_PERIOD3 	0x1FFF
	#define PRE_SCALE		64
	#define CLRWDT() __asm__ volatile("clrwdt");



/********************** Pin parameters *******************************/
    #define MAN_AFC_SELECT	_RD0
	#define MAN_DELAY_CTRL	_RB8
	#define	WARM_UP			_RB9
	#define	MAN_DELAY_UP	_RB4
	#define MAN_DELAY_DOWN	_RB5
	#define FLT1			_LATF15
	#define FLT2			_LATA10
	#define	FLT3			_LATA11	
	#define SAMPLE			_LATB10

/********************** New parameters *******************************/
	#define HOME_ADDRESS			(unsigned int)0x1200			//Define a fixed address for saving the delay
	#define MOTOR_POS_ADDRESS		(unsigned int)0x1900			//Define a fixed address for saving the motor position
	#define CHECK_ADDRESS			(unsigned int)0x0500			//Define a fixed address for checking if the delay has ever been set
	#define MAX_DELAY				0xFFF8
	#define MIN_DELAY				0x0008
	#define EEPROM_DELAY			145500 							/*	(unsigned int)(0.005*FCY), 5mS delay to allow EEPROM to write*/
	#define LARGE_ERROR				0x00FF
	#define SMALL_ERROR				0x000A
	#define GLITCH_ERROR			0x004B
	#define DELAY_CONST				1
	#define MAX_POS_CURRENT				0x23C							/* 1A on 0.1 Ohm */
	#define MAX_NEG_CURRENT				0x1C4							/* 1A on 0.1 Ohm */
	#define AFC_LOST_FAULT			0x0C
	#define OCFAULT					0x03
	#define BUFFER_SIZE				32
	
	#define COOL_SHIFTS				14
	#define MOTOR_PRE_SHIFTS		16
	#define MOTOR_RATE				800
	#define COOL_RATE				250
	#define MOTOR_POST_SHIFTS		9
	#define SMALL_THERMAL_ERROR		0x000F
/********************** Motor parameters *******************************/

    #define MOTOR_R                 6.2             //motor resistance in the configuration in which it's conected
    #define MOTOR_L                 0.004           //motor  inductance
    #define DC_BUS                  24.0            //board power supply
    
    #define RATED_MOTOR_CURRENT     0.67             //maximum desired motor current (rated motor current);
	#define MAX_STEPS				1000
	
	#define MOTOR_PHASE	  (unsigned int)(FCY*32/FPWM/2)

	#define FULL_DC 0xFFEF
	#define HALF_DC (unsigned int)(FCY*32/FPWM/2)  //23280 / 2 = 11640
	#define ZERO_DC 0
	#define MOTOR_VOLTAGE (unsigned int)0x4000  //70% DC
	#define MOTOR_VOLTAGE_LOW (unsigned int)0x51D8 //90% DC
/******************** Software Parameters ****************************/  

  
    //Decay driving mode
    #define FIXED_DECAY             0               //in this mode only one current decay mode is used
    #define ALTERNATE_DECAY         1               //in this mode the current decay is alternated between two modes
    
    #define TABLE_SIZE	            128             //sinewave look-up table size
    #define TABLE_SIZE_MUL2	        TABLE_SIZE*2    //table size *2
    #define TABLE_SIZE_MUL3	        TABLE_SIZE*3    //table size *3
    #define TABLE_SIZE_MUL4	        TABLE_SIZE*4    //table size *4
    
    //Step sizes
    #define ST_FULLSTEP             0b1100000               //possible values for stepSize variable
    #define ST_HALFSTEP             0b1110000
    #define ST_1_4STEP              0b1111000
    #define ST_1_8STEP              0b1111100
    #define ST_1_16STEP             0b1111110
    #define ST_1_32STEP             0b1111111
    
    
    
    
    //define bipolar decay modes                
                                                    //drive = high mosfet + opposite low mosfet
    #define D_FAST                  0               //fast decay = all MOSFETS off
    #define D_SLOW_L_DIODE          1               //slow decay = low diode + opposite low mosfet
    #define D_SLOW_H_DIODE          2               //slow decay = high mosfet + opposite high diode
    #define D_SLOW_L_MOSFET         3               //slow decay = low mosfet + opposite low mosfet
    #define D_SLOW_H_MOSFET         4               //slow decay = high mosfet + opposite high mosfet
    #define D_REVERSE               5               //reverse = low mosfet + opposite high mosfet
   
    //State machine defines
    #define STATE_WARM_UP           1
    #define STATE_MAN               2
	#define STATE_AFC               3
	#define STATE_FAULT             4
    #define STATE_INIT              0
   
    
    //motor and winding direction - do not change - these values are used in different mathematical formulae
    #define FORWARD                 0               //winding forward direction; used by setDir variables
    #define REVERSE                 1               //winding reverse direction; used by setDir variables
	#define STOP					2
    #define MOTOR_FORWARD           1               //rotor is rotating in forward direction
    #define MOTOR_REVERSE          -1               //rotor is rotating in reverse direction
  
 
  
#endif
