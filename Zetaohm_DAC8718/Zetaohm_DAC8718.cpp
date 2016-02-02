#include <SPI.h>
#include <Zetaohm_DAC8718.h>


Zetaohm_DAC8718::Zetaohm_DAC8718() {
}

void Zetaohm_DAC8718::begin(uint8_t cs_pin, uint8_t ldac_pin) {
  _cs_pin = cs_pin;
  _ldac_pin = ldac_pin;
  pinMode(_cs_pin, OUTPUT);                        // cs_pin is also the SYNC pin
  pinMode(_ldac_pin, OUTPUT);                        // configure cs_pin as output
  digitalWriteFast(_cs_pin, HIGH);  //deactivate DAC
  digitalWriteFast(_ldac_pin, LOW); 
  //set gain register to 4x gain
    SPI.begin ();
    delay(10);
    softwareReset();
  //  configureGain(1);
}

void Zetaohm_DAC8718::enableSCE(bool value){
	uint8_t buffer[3];

	buffer[0] = 0;				 	// set config register address
	buffer[1] = 1 << 2;				// set reset bit D13
	buffer[2] = 0;						// set second byte of output level

	transmitSPI(buffer);
}

void Zetaohm_DAC8718::softwareReset(){
	uint8_t buffer[3];

	buffer[0] = 0;				 	// set config register address
	buffer[1] = 1 << 5;				// set reset bit D13
	buffer[2] = 0;						// set second byte of output level

	transmitSPI(buffer);
}

void Zetaohm_DAC8718::configureGain(bool gain){
	uint8_t buffer[3];

	buffer[0] = 0;
	buffer[1] = gain;			// set GAIN A buffer bit
	buffer[2] = (gain<<7); 	// set GAIN B buffer bit

	Serial.println("GAIN BUFFER BYTE 1: " + String(buffer[1]) + "\tBYTE 2: " + String(buffer[2]));
	transmitSPI(buffer);
}



void Zetaohm_DAC8718::setVoltage(uint8_t dac, uint16_t output)
{
  uint8_t buffer[3];

  buffer[0] = (0x01 << 3) + dac; 	// set address byte for DAC Update
  buffer[1] = output / 256;			// set first byte of output level
  buffer[2] = output % 256;			// set second byte of output level

  transmitSPI(buffer);
}

void Zetaohm_DAC8718::setGain(uint8_t dac, uint16_t gain ){
  uint8_t buffer[3];

  buffer[0] = (0x03 << 3) + dac; 	// set address byte for DAC Update
  buffer[1] = gain / 256;			// set first byte of output level
  buffer[2] = gain % 256;			// set second byte of output level

  transmitSPI(buffer);
}

void Zetaohm_DAC8718::setZero(uint8_t dac, uint16_t zero ){
  uint8_t buffer[3];

  buffer[0] = (0x01 << 4) + dac; 	// set address byte for DAC Update
  buffer[1] = zero / 256;			// set first byte of output level
  buffer[2] = zero % 256;			// set second byte of output level

  transmitSPI(buffer);
}

void Zetaohm_DAC8718::transmitSPI(uint8_t buffer[3]){
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWriteFast(_cs_pin, LOW);  // Begin transmission, bring CS line low
 
  SPI.transfer(buffer, 3 );  //  FRAME 1 COMMAND BYTE - Command + DAC Address (C3 C2 C1 C0 A3 A2 A1 A0)

  digitalWriteFast(_cs_pin, HIGH);  // End transmission, bring CS line high
  SPI.endTransaction();
}

