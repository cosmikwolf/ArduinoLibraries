/**************************************************************************/
/*! 
    @file     Zetaohm_AD5669.h
    @author   T. Kariya
		@license  BSD (see license.txt)

		@section  HISTORY
    v1.0  - First release
*/
/**************************************************************************/
#ifndef Zetaohm_AD5669_h_
#define Zetaohm_AD5669_h_

#include <i2c_t3.h>

class Zetaohm_AD5669{
 public:
  Zetaohm_AD5669();
  void begin(uint8_t a);  
  void setVoltage( uint16_t output, uint8_t dac );
	void internalReferenceEnable(boolean enable);
 private:
  uint8_t _i2caddr;
};

#endif