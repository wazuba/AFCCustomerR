#include "stepper.h"
#include "i2c.h"
#include <dsp.h>

/*----------------------------------------------------------------------------*/

/******************************************************************************
* Function:     initPeripherals()
*
* Output:		None
*
* Overview:		This function configures the I/O pins
*
* Note:			None
*******************************************************************************/

void initPeripherals(void){

	//TRIS A
	_TRISA8 = 0;
	_TRISA9 = 1;
	_TRISA10 = 0;
	_TRISA11 = 0;
	
	//LAT A
	_LATA8 = 0;
	FLT1 = 1;
	FLT2 = 1;
	
	//TRIS B
	TRISB = 0xFFF;
	_TRISB10 = 0;
	
	
	//LAT B
	SAMPLE = 0;
	
	//TRIS D
	_TRISD0 = 1;
	_TRISD1 = 0;
	
	//LAT D
	_LATD0 = 0;
	
	//TRIS E
	TRISE = 1;
	_TRISE0 = 0;
	_TRISE1 = 0;
	_TRISE2 = 0;
	_TRISE3 = 0;
	_TRISE4 = 0;
	_TRISE5 = 0;
	_TRISE6 = 0;
	_TRISE7 = 0;
	
	//LAT E
	_LATE0 = 0;
	_LATE1 = 0;
	_LATE2 = 0;
	_LATE3 = 0;
	_LATE4 = 0;
	_LATE5 = 0;
	_LATE6 = 0;
	_LATE7 = 0;
	
	//TRIS F
	TRISF = 0;
	_TRISF6 = 1;
	
	//LAT F
	FLT3 = 1;


	homeAddress = HOME_ADDRESS;
	motorPosAddress = MOTOR_POS_ADDRESS;
	motorStep = TIMER_PERIOD2;
	stepSize = ST_1_32STEP;
	sampleTrigger = 0;
	faultStatus.ocFault = 0;
	faultStatus.afcLostFault = 0;
	faultStatus.cpuFault = 0;
	faultStatus.warmUpFault = 0;
	count =0;
	error = 0;
	/* Init I2C */
	I2CCON = I2C_ON & I2C_IDLE_CON &  I2C_CLK_REL & I2C_IPMI_DIS & I2C_7BIT_ADD & I2C_SLW_DIS & I2C_SM_DIS; 
	I2CCON &= I2C_GCALL_DIS & I2C_STR_DIS & I2C_ACK;
	I2CBRG = I2C_BAUD_RATE_GENERATOR;

	thermalCounter = 0;
	counterA = 0;
	counterB = 0;
}    

/******************************************************************************
* Function:     initInterrupts()
*
* Output:		None
*
* Overview:		This function initializes the External Interrupts
*
* Note:			None
*******************************************************************************/

void initInterrupts(void)
{
	/* Set up external INT0 */
	IEC0bits.INT0IE = 0;		/* Disable INT0 Interrupt */
	IFS0bits.INT0IF = 0;		/* Clear Interrupt flag */
	INTCON2bits.INT0EP = 1; 	/* Interrupt on falling edge */ 
	IPC0bits.INT0IP = 7;		/* Set interrupt to highest priority */
	triggerINT = 0;
	
	/* Set up external INT2 */
	IEC1bits.INT2IE = 1;		/* Enable INT2 Interrupt */
	IFS1bits.INT2IF = 0;		/* Clear Interrupt flag */
	INTCON2bits.INT2EP = 1; 	/* Interrupt on falling edge */ 
	IPC4bits.INT2IP = 6;		/* Set interrupt to second highest priority */
	
	IEC1bits.PSEMIE = 1;
	IFS1bits.PSEMIF = 0;
	IPC4bits.PSEMIP = 2;
	IPC1bits.T2IP = 3;
}

/******************************************************************************
* Function:     initPWM()
*
* Output:		None
*
* Overview:		This function initializes the PWM
*
* Note:			None
*******************************************************************************/

void initPWM(void)
{
	
	PTPER = FCY*32/FPWM;        	/* PTPER = FCY*32(PLL)/(Desired PWM Freq.) Refer to PWM section for more details   */
	/* Initialize PWM Generator 1 */
	
	IOCON1bits.PENH		= 1;        /* PWM Module controls High output */
	IOCON1bits.PENL		= 1;        /* PWM Module controls Low output */
	IOCON1bits.POLH		= 0;        /* High Output Polarity is active High */
	IOCON1bits.POLL		= 0;        /* Low Output Polarity is active High */
	IOCON1bits.PMOD		= 0;        /* Comp. output mode */
	IOCON1bits.OVRENH 	= 0;        /* High Output Override Enabled */
	IOCON1bits.OVRENL 	= 0;        /* Low Output Override Enabled */
	IOCON1bits.OVRDAT   = 0;
	IOCON1bits.OSYNC   = 1;
	

	PWMCON1bits.FLTSTAT = 0;        /* Clear Fault Interrupt flag */
	PWMCON1bits.CLSTAT = 0;         /* Clear Current Limit Interrupt flag */
	PWMCON1bits.TRGSTAT = 0;        /* Clear PWM Trigger Interrupt flag */
	PWMCON1bits.FLTIEN = 0;         /* Disable Fault Interrupt */
	PWMCON1bits.CLIEN = 0;          /* Disable Current Limit Interrupt */
	PWMCON1bits.TRGIEN = 0;         /* Disable Trigger Interrupt */
	PWMCON1bits.ITB	= 0;            /* Time base is read from PTMR */
	PWMCON1bits.MDCS = 0;           /* Duty cycle is read from PDC */
	DTR1 = DEADTIME;
	ALTDTR1 = DEADTIME;
	PWMCON1bits.XPRES = 0;          /* No extenal reset for PTMR */
	PWMCON1bits.IUE = 1;            /* Immediate update to PDC */
										
	PDC1 = HALF_DC;                     
	PHASE1 = 0;                     /* No staggering */
		/* Initialize PWM Generator 2 */
	IOCON2bits.PENH		= 1;        /* PWM Module controls High output */
	IOCON2bits.PENL		= 1;        /* PWM Module controls Low output */
	IOCON2bits.POLH		= 0;        /* High Output Polarity is active High */
	IOCON2bits.POLL		= 0;        /* Low Output Polarity is active High */
	IOCON2bits.PMOD		= 0;        /* Comp. output mode */
	IOCON2bits.OVRENH 	= 0;        /* High Output Override Enabled */
	IOCON2bits.OVRENL 	= 0;        /* Low Output Override Enabled */
	IOCON2bits.OVRDAT   = 0;
	IOCON2bits.OSYNC   = 1;

	PWMCON2bits.FLTSTAT = 0;        /* Clear Fault Interrupt flag */
	PWMCON2bits.CLSTAT = 0;         /* Clear Current Limit Interrupt flag */
	PWMCON2bits.TRGSTAT = 0;        /* Clear PWM Trigger Interrupt flag */
	PWMCON2bits.FLTIEN = 0;         /* Disable Fault Interrupt */
	PWMCON2bits.CLIEN = 0;          /* Disable Current Limit Interrupt */
	PWMCON2bits.TRGIEN = 0;         /* Disable Trigger Interrupt */
	PWMCON2bits.ITB	= 0;            /* Time base is read from PTMR */
	PWMCON2bits.MDCS = 0;           /* Duty cycle is read from PDC */
	DTR2 = DEADTIME;
	ALTDTR2 = DEADTIME;
	PWMCON2bits.XPRES = 0;          /* No extenal reset for PTMR */
	PWMCON2bits.IUE = 1;            /* Immediate update to PDC */
										
	PDC2 = HALF_DC;                     
	PHASE2 = 0;                     /* No staggering */
		/* Initialize PWM Generator 3 */
	IOCON3bits.PENH		= 1;        /* PWM Module controls High output */
	IOCON3bits.PENL		= 1;        /* PWM Module controls Low output */
	IOCON3bits.POLH		= 0;        /* High Output Polarity is active High */
	IOCON3bits.POLL		= 0;        /* Low Output Polarity is active High */
	IOCON3bits.PMOD		= 0;        /* Comp. output mode */
	IOCON3bits.OVRENH 	= 0;        /* High Output Override Enabled */
	IOCON3bits.OVRENL 	= 0;        /* Low Output Override Enabled */
	IOCON3bits.OVRDAT   = 0;
	IOCON3bits.OSYNC   = 1;

	PWMCON3bits.FLTSTAT = 0;        /* Clear Fault Interrupt flag */
	PWMCON3bits.CLSTAT = 0;         /* Clear Current Limit Interrupt flag */
	PWMCON3bits.TRGSTAT = 0;        /* Clear PWM Trigger Interrupt flag */
	PWMCON3bits.FLTIEN = 0;         /* Disable Fault Interrupt */
	PWMCON3bits.CLIEN = 0;          /* Disable Current Limit Interrupt */
	PWMCON3bits.TRGIEN = 0;         /* Disable Trigger Interrupt */
	PWMCON3bits.ITB	= 0;            /* Time base is read from PTMR */
	PWMCON3bits.MDCS = 0;           /* Duty cycle is read from PDC */
	DTR3 = DEADTIME;
	ALTDTR3 = DEADTIME;
	PWMCON3bits.XPRES = 0;          /* No extenal reset for PTMR */
	PWMCON3bits.IUE = 1;            /* Immediate update to PDC */
										
	PDC3 = HALF_DC;                     
	PHASE3 = 0;                     /* No staggering */
		/* Initialize PWM Generator 4 */
	IOCON4bits.PENH		= 1;        /* PWM Module controls High output */
	IOCON4bits.PENL		= 1;        /* PWM Module controls Low output */
	IOCON4bits.POLH		= 0;        /* High Output Polarity is active High */
	IOCON4bits.POLL		= 0;        /* Low Output Polarity is active High */
	IOCON4bits.PMOD		= 0;        /* Comp. output mode */
	IOCON4bits.OVRENH 	= 0;        /* High Output Override Enabled */
	IOCON4bits.OVRENL 	= 0;        /* Low Output Override Enabled */
	IOCON4bits.OVRDAT   = 0;
	IOCON4bits.OSYNC   = 1;

	PWMCON4bits.FLTSTAT = 0;        /* Clear Fault Interrupt flag */
	PWMCON4bits.CLSTAT = 0;         /* Clear Current Limit Interrupt flag */
	PWMCON4bits.TRGSTAT = 0;        /* Clear PWM Trigger Interrupt flag */
	PWMCON4bits.FLTIEN = 0;         /* Disable Fault Interrupt */
	PWMCON4bits.CLIEN = 0;          /* Disable Current Limit Interrupt */
	PWMCON4bits.TRGIEN = 0;         /* Disable Trigger Interrupt */
	PWMCON4bits.ITB	= 0;            /* Time base is read from PTMR */
	PWMCON4bits.MDCS = 0;           /* Duty cycle is read from PDC */
	DTR4 = DEADTIME;
	ALTDTR4 = DEADTIME;
	PWMCON4bits.XPRES = 0;          /* No extenal reset for PTMR */
	PWMCON4bits.IUE = 1;            /* Immediate update to PDC */
										
	PDC4 = HALF_DC;                     
	PHASE4 = 0;                     /* No staggering */	
	_SEVTCMP	=	10;
	PTCONbits.SEIEN = 1;
	PTCONbits.PTEN = 1;                /* Enable PWM Module */

}

/******************************************************************************
* Function:     initADC()
*
* Output:		None
*
* Overview:		This function initializes the ADC
*
* Note:			None
*******************************************************************************/

void initADC(void)
{
		
	ADCONbits.ADSIDL    = 0;        /* Operate in Idle Mode */
	ADCONbits.FORM      = 0;        /* Output in Integer Format	*/
	ADCONbits.EIE       = 0;        /* No Early Interrupt */
	ADCONbits.ORDER     = 0;        /* Even channel first */
	ADCONbits.SEQSAMP   = 1;        /* Sequential Sampling Enabled */
	ADCONbits.ADCS      = 5;        /* Clock Divider is set up for Fadc/14 */
	IPC2bits.ADIP = 4; /* Set ADC Interrupt Priority*/
	ADPCFG = 0xF5F0;				/* Set AN0, AN1, AN2, AN3, AN9 and AN11 as analog inputs */
	ADSTAT = 0;                     /* Clear the ADSTAT register */
	ADCPC0bits.TRGSRC0 	= 1;      	/* Trigger conversion of AN0 and AN1 Software Trigger */
	ADCPC0bits.TRGSRC1 	= 12;      /* Trigger conversion of AN2 and AN3 on Timer 1 period match */
	ADCPC2bits.TRGSRC5	= 1;      	/* Trigger conversion of AN11 Software Trigger */
	ADCPC2bits.TRGSRC4	= 1;      	/* Trigger conversion of AN9 Software Trigger */
	ADCPC0bits.IRQEN0	= 1;        /* Enable the interrupt */
	ADCPC0bits.IRQEN1	= 1;        /* Enable the interrupt */
	ADCPC2bits.IRQEN5	= 1;        /* Enable the interrupt */
	ADCPC2bits.IRQEN4	= 1;        /* Enable the interrupt */
		
	/* Set up the Interrupts */
	
	IFS0bits.ADIF 	= 0;            /* Clear AD Interrupt Flag */	
	IEC0bits.ADIE	= 1;
	IFS0bits.T1IF	= 0;            /* Clear Timer1 Interrupt Flag */
	IEC0bits.T1IE	= 0;            /* Timer Interrupt is not needed */
	

	/* Enable ADC */
	ADCONbits.ADON 	= 1;            /* Start the ADC module */		
	
	
}



/******************************************************************************
* Function:     initTMR()
*
* Output:		None
*
* Overview:		The function initializes timer 1,2 and 3 as 16 bit timers
*
* Note:			None
*******************************************************************************/

void initTMR(void)
{	
	
	/* Set up Timer1 */
	T1CON 	= 0;                    /* Timer with 0 prescale */
	T1CONbits.TCKPS = 0;
	TMR1	= 0;                    /* Clear the Timer counter */
	PR1		= TIMER_PERIOD;         /* Load the period register */
	IFS0bits.T1IF	= 0;            /* Clear Timer1 Interrupt Flag */
	T1CONbits.TON = 1;
	
	
	/* Set up Timer2 */
	T2CON 	= 0;                    /* Timer with 0 prescale */
	T2CONbits.TCKPS = 2;
	TMR2	= 0;                    /* Clear the Timer counter */
	PR2		= TIMER_PERIOD2;         /* Load the period register */
	IFS0bits.T2IF	= 0;            /* Clear Timer2 Interrupt Flag */
	IEC0bits.T2IE	= 1;
	
	/* Set up Timer3 */
	T3CON 	= 0;                    /* Timer with 0 prescale */
	T3CONbits.TCKPS = 0;
	TMR3	= 0;                    /* Clear the Timer counter */
	PR3		= TIMER_PERIOD3;         /* Load the period register */
	IFS0bits.T3IF	= 0;            /* Clear Timer Interrupt Flag */
	IEC0bits.T3IE	= 1;
}

/*
EOF
*/


