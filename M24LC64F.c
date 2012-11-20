#include "M24LC64F.h"
#include "libpic30.h"
#include "stepper.h"

void ByteWriteI2C(unsigned char HighAdd, unsigned char LowAdd, unsigned char data,  unsigned char address);

unsigned char ByteReadI2C(unsigned char HighAdd, unsigned char LowAdd, unsigned char address);



void M24LC64FWriteWord(unsigned int register_location, unsigned int data, unsigned char address) {
  unsigned int temp;
  unsigned char adr_high_byte;
  unsigned char adr_low_byte;
  unsigned char data_low_byte;
  unsigned char data_high_byte;

  if (register_location <= M24LC64F_MAX_16BIT_REGISTERS) {
    // The register_location is in "word" locations and if greater than M24LC64F_MAX_16BIT_REGISTERS it will extended beyond the device
    
    data_high_byte = (data >> 8);
    data_low_byte = (data & 0x00FF);
    
    temp = (register_location*2);
    adr_high_byte = (temp >> 8);
    adr_low_byte = (temp & 0x00FF);
    ByteWriteI2C(adr_high_byte, adr_low_byte, data_high_byte, address);
    
    temp = (register_location*2+1);
    adr_high_byte = (temp >> 8);
    adr_low_byte = (temp & 0x00FF);
    ByteWriteI2C(adr_high_byte, adr_low_byte, data_low_byte, address);
    
  }
}



unsigned int M24LC64FReadWord(unsigned int register_location, unsigned char address) {
  unsigned int temp;
  unsigned char adr_high_byte;
  unsigned char adr_low_byte;
  unsigned char data_low_byte;
  unsigned char data_high_byte;
  
  if (register_location <= M24LC64F_MAX_16BIT_REGISTERS) {
    // The register_location is in "word" locations and if greater than M24LC64F_MAX_16BIT_REGISTERS it will extended beyond the device
    
    temp = (register_location*2);
    adr_high_byte = (temp >> 8);
    adr_low_byte = (temp & 0x00FF);
    data_high_byte = ByteReadI2C(adr_high_byte, adr_low_byte, address); 
    
    temp = (register_location*2+1);
    adr_high_byte = (temp >> 8);
    adr_low_byte = (temp & 0x00FF);
    data_low_byte = ByteReadI2C(adr_high_byte, adr_low_byte, address); 
    
  } else {
    data_low_byte = 0;
    data_high_byte = 0;
  }
  
  temp = data_high_byte*256 + data_low_byte;
  return temp;
}




void ByteWriteI2C(unsigned char HighAdd, unsigned char LowAdd, unsigned char data, unsigned char address) {
  unsigned char eeprom_busy;
  
	TMR3 = 0;
	T3CONbits.TON 	= 1;
    eeprom_busy = 1;
    while (I2CSTATbits.TRSTAT);		//Wait for bus Idle
    
    I2CCONbits.SEN = 1;		        //Generate Start COndition
    while (I2CCONbits.SEN);	                //Wait for Start COndition
    TMR3 = 0;
    I2CTRN = (address | M24LC64F_WRITE_CONTROL_BIT);		//Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3 = 0;
    while (I2CSTATbits.TRSTAT);		//Cleared at end of Slave ACK
	TMR3 = 0;
        
    I2CTRN = HighAdd;		                //Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3 = 0;
    while (I2CSTATbits.TRSTAT);		//Cleared at end of Slave ACK
	TMR3 = 0;
        
    I2CTRN = LowAdd;		                //Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3 = 0;
    while (I2CSTATbits.TRSTAT);		//Cleared at end of Slave ACK
	TMR3 = 0;
    
    I2CTRN = data;		                //Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3 = 0;
    while (I2CSTATbits.TRSTAT);		//Cleared at end of Slave ACK
	TMR3 = 0;
    
    I2CCONbits.PEN = 1;		        //Generate Stop Condition
    while (I2CCONbits.PEN);	                //Wait for Stop Condition
	TMR3 = 0;
    
    while (I2CSTATbits.TRSTAT);		//Wait for bus Idle
	TMR3 = 0;
	T3CONbits.TON 	= 0;
    __delay32(EEPROM_DELAY);
    T3CONbits.TON 	= 1;
    while(eeprom_busy) {
      I2CCONbits.SEN = 1;		        //Generate Start COndition
	  TMR3 = 0;
      while (I2CCONbits.SEN);	                //Wait for Start COndition
	  TMR3 = 0;
		
      
      I2CTRN = (address | M24LC64F_WRITE_CONTROL_BIT);		//Load byte to I2C Transmit buffer
      while (!I2CSTATbits.TRSTAT);	        //Set when transmit process starts
	  TMR3 = 0;
      while (I2CSTATbits.TRSTAT);	        //Cleared at end of Slave ACK
      TMR3 = 0;
      if (!I2CSTATbits.ACKSTAT) {              //ACK means that the write process is complete
	eeprom_busy = 0;
	I2CCONbits.PEN = 1;		        //Generate Stop Condition
	while (I2CCONbits.PEN);	        //Wait for Stop Condition
	TMR3 = 0;
	T3CONbits.TON 	= 0;
      }
    }
	
}




unsigned char ByteReadI2C(unsigned char HighAdd, unsigned char LowAdd, unsigned char address) {
  unsigned char data;
  
	TMR3 = 0;
	T3CONbits.TON 	= 1;
    while (I2CSTATbits.TRSTAT);		//Wait for bus Idle
	TMR3 = 0;
    I2CCONbits.SEN = 1;		        //Generate Start COndition
    while (I2CCONbits.SEN);	                //Wait for Start COndition
	TMR3 = 0;
    
    I2CTRN = (address | M24LC64F_WRITE_CONTROL_BIT);		//Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3 = 0;
    while (I2CSTATbits.TRSTAT);		//Cleared at end of Slave ACK
	TMR3 = 0;
    
    I2CTRN = HighAdd;		                //Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3 = 0;
    while (I2CSTATbits.TRSTAT);		//Cleared at end of Slave ACK
	TMR3 = 0;
    
    I2CTRN = LowAdd;		                //Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3 = 0;
    while (I2CSTATbits.TRSTAT);		//Cleared at end of Slave ACK
	TMR3 = 0;
    
    I2CCONbits.RSEN = 1;		        //Generate Re-Start Condition
    while (I2CCONbits.RSEN);	                //Wait for Re-Start Condition
	TMR3 = 0;
    
    I2CTRN = (address | M24LC64F_READ_CONTROL_BIT);		//Load byte to I2C Transmit buffer
    while (!I2CSTATbits.TRSTAT);		//Set when transmit process starts
	TMR3 = 0;
    while (I2CSTATbits.TRSTAT);	        //Wait for bus Idle
	TMR3 = 0;
    
    I2CCONbits.RCEN = 1;			//Start Master receive
    while(I2CCONbits.RCEN);
	TMR3 = 0;
    
    I2CCONbits.PEN = 1;		        //Generate Stop Condition
    while (I2CCONbits.PEN);	                //Wait for Stop Condition
	TMR3 = 0;
    
    while(!I2CSTATbits.RBF);		        //Wait for receive bufer to be full
	TMR3 = 0;
    data = I2CRCV;
	T3CONbits.TON 	= 0;;
   return data;
}




