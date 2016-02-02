/**************************************************************************/
/*! 
    @file     Zetaohm_AD5676.h
    @author   T. Kariya
		@license  BSD (see license.txt)

		@section  HISTORY
    v1.0  - First release
*/
/**************************************************************************/
#ifndef Zetaohm_AD5676_spi4teensy_h_
#define Zetaohm_AD5676_spi4teensy_h_

#include <spi4teensy3.h>
//#include <SPI.h>

class Zetaohm_AD5676{
 public:
  Zetaohm_AD5676();
  void begin(uint8_t cs_pin, uint8_t ldac_pin);  
  void setVoltage( uint16_t output, uint8_t dac );
  void softwareReset();
	void internalReferenceEnable(bool enable);
 private:
  uint8_t _cs_pin;   //AD5676 SYNC line
  uint8_t _ldac_pin;
};

#endif