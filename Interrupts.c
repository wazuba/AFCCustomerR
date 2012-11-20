
#include "stepper.h"
#include "tables.h"
/******************************************************************************
* Interrupt:     _ADCInterrupt()
*
* Output:		None
*
* Overview:		ISR for ADC, processes for RF signals and current sense
*
* Note:			None
*******************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt (void)
{
		IEC0bits.ADIE	= 0;
		IFS0bits.ADIF = 0; // Clear ADC Interrupt Flag
		IEC0bits.ADIE	= 1;
	
}

/******************************************************************************
* Interrupt:     _PWMSpEventMatchInterrupt()
*
* Output:		None
*
* Overview:		ISR for PWM special event, executes duty cycle updates
*
* Note:			None
*******************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _PWMSpEventMatchInterrupt(void)
{
	//int myTime = TMR2;
	counterA %= TABLE_SIZE;
	counterB += TABLE_SIZE>>2; //90 degree shift
	counterB %= TABLE_SIZE;
	if (T2CONbits.TON)
	{
		if (setDir == FORWARD)
		{
			//FLT1 = 0;
			PDC1 = winding1ATable[(counterA)&stepSize];
			PDC2 = winding1BTable[(counterA)&stepSize];
			PDC3 = winding1ATable[(counterB)&stepSize];
			PDC4 = winding1BTable[(counterB)&stepSize];
			//FLT1 = 1;

		}	
		if (setDir == REVERSE)
		{
			//FLT2 = 0;
			PDC1 = winding1ATable[(counterB)&stepSize];
			PDC2 = winding1BTable[(counterB)&stepSize];
			PDC3 = winding1ATable[(counterA)&stepSize];
			PDC4 = winding1BTable[(counterA)&stepSize];
			//FLT2 = 1;
		}	
		if (setDir == STOP)
		{
			/*PWM1 and PWM3 are at 90% DC and PWM2 and PWM4 are at ~100% DC this is an effective voltage of ~Vbus*(1-0.9) = 2.4V*/
			PDC1 = MOTOR_VOLTAGE_LOW;
			PDC2 = FULL_DC;
			PDC3 = MOTOR_VOLTAGE_LOW;
			PDC4 = FULL_DC;
		}
		
		counterA++;
		counterB++;
	}
	IFS1bits.PSEMIF = 0;
	if (thermalCounter < 2500)
		thermalCounter++;
	else
		thermalCounter = 0;
}
/******************************************************************************
* Interrupt:     _INT0Interrupt()
*
* Output:		None
*
* Overview:		ISR for external interrupt INT0, triggers on rising edge of incoming
*				trigger (falling egde of inverted signal).
* Note:			None
*******************************************************************************/
void __attribute__((interrupt, shadow, no_auto_psv)) _INT0Interrupt(void)
{
	
	IEC0bits.INT0IE = 0;  /* disable interrupt to prevent debouncing */
	
	//__delay32(triggerDelay); /* triggerDelay  */
/*	__asm__ volatile("mov.w  _triggerDelay,w0");
	__asm__ volatile("mov.w #0x0,w1");
	__asm__ volatile("sub #1023,w0");
	__asm__ volatile("subb #0,w1");
	__asm__ volatile("bra lt,2f");
	__asm__ volatile("4: repeat #1006");
	__asm__ volatile("nop");
	__asm__ volatile("sub #1012, w0");
	__asm__ volatile("subb #0, w1");
	__asm__ volatile("bra ge, 4b");
	__asm__ volatile("add #1,w0");
	__asm__ volatile("2: add #1010,w0");
	__asm__ volatile("bra lt,3f");
	__asm__ volatile("repeat w0");
	__asm__ volatile("nop");
	__asm__ volatile("3:");*/
	
	PTCONbits.PTEN = 0;                /* Disable PWM Module */	
	
	SAMPLE = 1;
	
	//__delay32(10);
	__asm__ volatile("mov.w  #0x10,w0");
	__asm__ volatile("mov.w #0x0,w1");
	__asm__ volatile("sub #1023,w0");
	__asm__ volatile("subb #0,w1");
	__asm__ volatile("bra lt,2f");
	__asm__ volatile("4: repeat #1006");
	__asm__ volatile("nop");
	__asm__ volatile("sub #1012, w0");
	__asm__ volatile("subb #0, w1");
	__asm__ volatile("bra ge, 4b");
	__asm__ volatile("add #1,w0");
	__asm__ volatile("2: add #1010,w0");
	__asm__ volatile("bra lt,3f");
	__asm__ volatile("repeat w0");
	__asm__ volatile("nop");
	__asm__ volatile("3:");
	SAMPLE = 0;
	
	triggerINT = 0;
	IFS0bits.INT0IF = 0;
	sampleTrigger = 1;

	PTCONbits.PTEN = 1;                /* Enable PWM Module */

}

/******************************************************************************
* Interrupt:     _INT2Interrupt()
*
* Output:		None
*
* Overview:		ISR for external interrupt INT2, triggers on falling edge 
*
* Note:			None
*******************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void)
{
	if (state != STATE_INIT)
		{
		state = STATE_FAULT;
		faultStatus.ocFault = 1;
		IEC1bits.INT2IE = 0;  /* disable interrupt to prevent debouncing */
		
		}
		
	IFS1bits.INT2IF	= 0;

}

/******************************************************************************
* Interrupt:     _T2Interrupt()
*
* Output:		None
*
* Overview:		
*
* Note:			None
*******************************************************************************/
void __attribute__ ((interrupt, no_auto_psv)) _T2Interrupt(void)
{
	IFS0bits.T2IF	= 0;
	T2CONbits.TON 	= 0;
	count++;
	/*flagT2 = 1;*/
	/*PDC1 = HALF_DC;
	PDC2 = HALF_DC;
	PDC3 = HALF_DC;
	PDC4 = HALF_DC;*/
}

/******************************************************************************
* Interrupt:     _T3Interrupt()
*
* Output:		None
*
* Overview:		I2C Timeout 
*
* Note:			None
*******************************************************************************/
void __attribute__ ((interrupt, no_auto_psv)) _T3Interrupt(void)
{
	IFS0bits.T3IF	= 0;
	__asm__ volatile ("reset");
}
/*EOF*/
