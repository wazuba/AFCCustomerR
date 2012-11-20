#ifndef __M24LC64F_H
#define __M24LC64F_H

/*
  This module is designed to inteface a 16 bit microchip processor with the 8 bit flash EEPROM.
  This module usees the function M24LC64FWriteWord and M24LC64FReadWord to treat the EEPROM as if it were a 16 bit device.
  This makes working with the 16 bit processor much easier.
  
  The devices is treated as a single block of 4K 16bit registers.

  Only I2C Buses 1 and 2 are supported at this time.
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

// ---------- Configure For the M24LC64F Addresses  ----------------- //
#define M24LC64F_READ_CONTROL_BIT          0b00000001
#define M24LC64F_WRITE_CONTROL_BIT         0b00000000

#define M24LC64F_ADDRESS_0                 0b10100000
#define M24LC64F_ADDRESS_1                 0b10100010
#define M24LC64F_ADDRESS_2                 0b10100100
#define M24LC64F_ADDRESS_3                 0b10100110
#define M24LC64F_ADDRESS_4                 0b10101000
#define M24LC64F_ADDRESS_5                 0b10101010
#define M24LC64F_ADDRESS_6                 0b10101100
#define M24LC64F_ADDRESS_7                 0b10101110

#define M24LC64F_MAX_16BIT_REGISTERS       0x1F3F
#define M24LC64F_WP_PIN_WRITE_ENABLE       0

void M24LC64FWriteWord(unsigned int register_location, unsigned int data, unsigned char address);

unsigned int M24LC64FReadWord(unsigned int register_location, unsigned char address);


#endif
