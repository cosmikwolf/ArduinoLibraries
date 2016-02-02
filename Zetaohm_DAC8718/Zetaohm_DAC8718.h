/**************************************************************************/
/*! 
    @file     Zetaohm_DAC8718.h
    @author   T. Kariya
	@license  BSD (see license.txt)

		@section  HISTORY
    v1.0  - First release
*/
/**************************************************************************/
#ifndef Zetaohm_DAC8717_h_
#define Zetaohm_DAC8718_h_
#include <SPI.h>

class Zetaohm_DAC8718{
 public:
  Zetaohm_DAC8718();
  void begin(uint8_t cs_pin, uint8_t ldac_pin);  
  void setVoltage(uint8_t dac, uint16_t output );
  void setGain(uint8_t dac, uint16_t gain);
  void setZero(uint8_t dac, uint16_t zero);
  void softwareReset();
  void configureGain(bool gain);
  void enableSCE(bool value);
 private:
  void transmitSPI(uint8_t buffer[3]);
  uint8_t _cs_pin;
  uint8_t _ldac_pin;
};

#endif