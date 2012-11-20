#include "MCP4725F.h"


void MCP4725FWriteWord(unsigned int data, unsigned char address) {

  unsigned char data_low_byte;
  unsigned char data_high_byte;

	 data = (data & 0x0FFF);
    data_high_byte = (data >> 8);
    data_low_byte = (data & 0x00FF);
   
	TMR3	= 0;
	T3CONbits.TON 	= 1;
    while (I2CSTATbits.TRSTAT);		//Wait for bus Idle
    TMR3	= 0;
    I2CCONbits.SEN = 1;		        //Generate Start COndition
    while (I2CCONbits.SEN);	                //Wait for Start COndition
    TMR3	= 0;
    I2CTRN = (address | MCP4725F_WRITE_CONTROL_BIT);		//Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3	= 0;
    while (I2CSTATbits.TRSTAT);		//Cleared at end of Slave ACK
	TMR3	= 0;
        
    I2CTRN = data_high_byte;		                //Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3	= 0;
    while (I2CSTATbits.TRSTAT);		//Cleared at end of Slave ACK
	TMR3	= 0;
        
    I2CTRN = data_low_byte;		                //Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3	= 0;
    while (I2CSTATbits.TRSTAT);		//Cleared at end of Slave ACK
	TMR3	= 0;

	I2CTRN = data_high_byte;		                //Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3	= 0;
    while (I2CSTATbits.TRSTAT);		//Cleared at end of Slave ACK
	TMR3	= 0;
        
    I2CTRN = data_low_byte;		                //Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3	= 0;
    while (I2CSTATbits.TRSTAT);		//Cleared at end of Slave ACK
	TMR3	= 0;
        
    I2CCONbits.PEN = 1;		        //Generate Stop Condition
    while (I2CCONbits.PEN);	                //Wait for Stop Condition
    
	T3CONbits.TON 	= 0;
    
}



unsigned int MCP4725FReadWord(unsigned char address) {
  unsigned int temp;
  unsigned char controlByte;
  unsigned char dataByte1;
  unsigned char dataByte2;
  TMR3	= 0;
  T3CONbits.TON 	= 1;
  
  while (I2CSTATbits.TRSTAT);		//Wait for bus Idle

    I2CCONbits.SEN = 1;		        //Generate Start COndition
    while (I2CCONbits.SEN);	                //Wait for Start COndition
	TMR3	= 0;
    
    I2CTRN = (address | MCP4725F_READ_CONTROL_BIT);		//Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3	= 0;
    while (I2CSTATbits.TRSTAT);	        //Wait for bus Idle
	TMR3	= 0;
    
    I2CCONbits.RCEN = 1;			//Start Master receive
    while(I2CCONbits.RCEN);
	TMR3	= 0;
      
    while(!I2CSTATbits.RBF);		        //Wait for receive buffer to be full
	TMR3	= 0;
    controlByte = I2CRCV;
	
	
	
	I2CCONbits.RCEN = 1;			//Start Master receive
    while(I2CCONbits.RCEN);
	TMR3	= 0;
	
	while(!I2CSTATbits.RBF);		        //Wait for receive buffer to be full
	TMR3	= 0;
    dataByte1 = I2CRCV;
	
	    
	I2CCONbits.RCEN = 1;			//Start Master receive
    while(I2CCONbits.RCEN);
	TMR3	= 0;
	
	while(!I2CSTATbits.RBF);		        //Wait for receive buffer to be full
	TMR3	= 0;
    dataByte2 = I2CRCV;
	
	I2CCONbits.RCEN = 1;			//Start Master receive
    while(I2CCONbits.RCEN);
	TMR3	= 0;
	
	while(!I2CSTATbits.RBF);		        //Wait for receive buffer to be full
	TMR3	= 0;
    dataByte2 = I2CRCV;
	
	I2CCONbits.RCEN = 1;			//Start Master receive
    while(I2CCONbits.RCEN);
	TMR3	= 0;
	
	while(!I2CSTATbits.RBF);		        //Wait for receive buffer to be full
	TMR3	= 0;
    dataByte1 = I2CRCV;
	
	I2CCONbits.PEN = 1;		        //Generate Stop Condition
    while (I2CCONbits.PEN);	                //Wait for Stop Condition
	TMR3	= 0;
    
	temp = (dataByte2&0x0F)*256 + dataByte1;
	T3CONbits.TON 	= 0;
  return temp;
}





