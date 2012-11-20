#ifndef __MCP4725F_H
#define __MCP4725F_H

/*
  This module is designed to inteface a 16 bit microchip processor with the 12 bit DAC.
  
 */

/*
  This module requires that the I2C bus be initialized externally before calling any of the read or write functions.
  The exact numbers will vary based on the frequency of your OSC and the Frequency that you want the bus to run at.
  Initialization should look something like this.
  Initialization will vary slightly between 24,30, and 33 series chips.
  See i2c.h for more information

  // I2Cx Initialization
  #define FCY_CLK                    40000000
  #define I2C1_CLK                   100000
  #define I2C2_CLK                   100000
  #define I2C1_BAUD_RATE_GENERATOR   ((FCY_CLK/I2C1_CLK) - (FCY_CLK/7690000) - 2)
  #define I2C2_BAUD_RATE_GENERATOR   ((FCY_CLK/I2C2_CLK) - (FCY_CLK/7690000) - 2)
  
  I2C1CON = I2C1_ON & I2C1_IDLE_CON &  I2C1_CLK_REL & I2C1_IPMI_DIS & I2C1_7BIT_ADD & I2C1_SLW_EN & I2C1_SM_DIS; 
  I2C1CON &= I2C1_GCALL_DIS & I2C1_STR_DIS & I2C1_ACK & I2C1_ACK_DIS;
  I2C1BRG = I2C1_BAUD_RATE_GENERATOR;

*/
#include "i2c.h"

// ---------- Configure For the MCP4725F Addresses  ----------------- //
#define MCP4725F_READ_CONTROL_BIT          0b00000001
#define MCP4725F_WRITE_CONTROL_BIT         0b00000000

#define MCP4725F_ADDRESS_0                 0b11000000
#define MCP4725F_ADDRESS_1                 0b11000010
#define MCP4725F_ADDRESS_2                 0b11000100
#define MCP4725F_ADDRESS_3                 0b11000110
#define MCP4725F_ADDRESS_4                 0b11001000
#define MCP4725F_ADDRESS_5                 0b11001010
#define MCP4725F_ADDRESS_6                 0b11001100
#define MCP4725F_ADDRESS_7                 0b11001110


void MCP4725FWriteWord(unsigned int data, unsigned char address);

unsigned int MCP4725FReadWord(unsigned char address);


#endif
