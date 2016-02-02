/**************************************************************************/
/*! 
    @file     Zetaohm_AD5669.h
    @author   T. Kariya
		@license  BSD (see license.txt)

		@section  HISTORY
    v1.0  - First release
*/
/**************************************************************************/

#include <i2c_t3.h>
#include <Zetaohm_AD5669.h>

/**************************************************************************/
/*! 
    @brief  Instantiates a new AD5669R class
*/
/**************************************************************************/
Zetaohm_AD5669::Zetaohm_AD5669() {
}

/**************************************************************************/
/*! 
    @brief  Setups the HW
*/
/**************************************************************************/
void Zetaohm_AD5669::begin(uint8_t addr) {
  _i2caddr = addr;
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
}

 
/**************************************************************************/
/*! 
    @brief  Sets the output voltage to a fraction of source vref.  (Value
            can be 0..4095)
    @param[in]  output
                The 12-bit value representing the relationship between
                the DAC's input voltage and its output voltage.
    @param[in]  writeEEPROM
                If this value is true, 'output' will also be written
                to the MCP4725's internal non-volatile memory, meaning
                that the DAC will retain the current voltage output
                after power-down or reset.



    COMMAND
  - 			C3	C2	C1	C0		
  0x00		0		0		0		0			Write to Input Register n
  0x01		0		0		0		1			Update DAC Register n
  0x02		0		0		1		0			Write to Input Register n; update all (software LDAC)
  0x03		0		0		1		1			Write to and update DAC Channel n
  0x04		0		1		0		0			Power down/power up DAC
  0x05		0		1		0		1			Load clear code register
  0x06		0		1		1		0			Load LDAC register
  0x07		0		1		1		1			Reset (power on reset)
  0x08		1		0		0		0			Set up internal REF register
  0x09		1		0		0		1			Enable multiple byte mode


*/
/**************************************************************************/
void Zetaohm_AD5669::internalReferenceEnable(boolean enable){
    Wire.beginTransmission(_i2caddr);
    Wire.write( (0x08 << 4) );
    Wire.write( 0xff );
    if (enable ){
      Wire.write( 0x81 );
    } else {
      Wire.write( 0x00 ); 
    }
    Wire.endTransmission(I2C_STOP);
}

void Zetaohm_AD5669::setVoltage(uint16_t output, uint8_t dac )
{
 // uint8_t twbrback = TWBR;
  //TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  Wire.beginTransmission(_i2caddr); // addressbyte 
   // Serial.println("Sending Command Byte");
	Wire.write( (0x03 << 4) + dac ); 	//	FRAME 2 COMMAND BYTE - Command + DAC Address (C3 C2 C1 C0 A3 A2 A1 A0)
 //   Serial.println("Sending Most Significant Data Byte");
 	Wire.write(output / 256); 											//	FRAME 3 MOST SIGNIFICANT DATA BYTE
 //     Serial.println("Sendign Least Significant Byte");
	Wire.write(output % 256); 											//	FRAME 4 LEAST SIGNIFICANT DATA BYTE

  Wire.endTransmission(I2C_STOP);
 // TWBR = twbrback;
}