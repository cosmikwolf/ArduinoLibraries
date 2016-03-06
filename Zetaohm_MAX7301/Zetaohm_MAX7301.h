#ifndef _ZETAOHM_MAX7301_H_
#define _ZETAOHM_MAX7301_H_

#include <inttypes.h>
#include <SPI.h>//this chip needs SPI

#define DEBOUNCE_THRESHOLD 	20000

class max7301 {

public:
	max7301(const uint8_t csPin, uint32_t spispeed = SPI_CLOCK_DIV4);//for SPI transactions
	max7301();//For include inside other libraries
	virtual void 	begin(bool protocolInitOverride=false); //protocolInitOverride=true	will not init the SPI	
	void 			gpioPinMode(uint16_t mode);					//OUTPUT=all out,INPUT=all in,0xxxx=you choose
	uint16_t 		readAddress(byte addr);
	void			setSPIspeed(uint32_t spispeed);//for SPI trans0actions
	void			writeByte(byte addr, byte data);	
	void			init(uint8_t index, uint8_t pin);		// initialize a new button
	void			update();								// update the input buffer
	bool			fell(uint8_t index);					// was the button pressed since the last check?
	bool			rose(uint8_t index);					// was the button pressed since the last check?
	bool			pressed(uint8_t index);
	uint32_t		inputBuffer;
	uint32_t		fellBuffer;
	uint32_t		roseBuffer;
	uint8_t			indexMap[32];

	elapsedMicros 	debounceTimer;

private:
	uint32_t		_spiTransactionsSpeed;//for SPI transactions
    uint8_t 		_cs;
	uint8_t 		_adrs;
};

#endif