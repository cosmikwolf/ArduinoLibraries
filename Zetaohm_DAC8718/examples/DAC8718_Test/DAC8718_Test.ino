#include <SPI.h>
#include <Zetaohm_DAC8718.h>
#include <elapsedMillis.h>

Zetaohm_DAC8718 dac8718;
elapsedMillis counter;

IntervalTimer masterClock;
uint32_t masterClockInterval = 10;

void setup(){
  Serial.begin(9600);
  Serial.println("Initializing AD5676 Octal DAC");
  dac8718.begin(15,	14);
 // dac8718.softwareReset();
  dac8718.configureGain(1);
 // dac8718.enableSCE(true);
  masterClock.begin(masterClockFunc,masterClockInterval);
  delay(1000);
}

void loop(){

//	uint32_t f = micros() % 65536;
//
//	Serial.println("Setting DAC to: " + String(f));
// 	 for (int i=0; i<8; i++){
//		dac8718.setVoltage(i, f);
// 	 }
//	if (counter < 1000){
//		for (int i=0; i<8; i++){
//			dac8718.setVoltage(i, 32000);
//		}
//	//	Serial.println("LOW");
//
//	} else {
//		for (int i=0; i<8; i++){
//			dac8718.setVoltage(i, 65536);
//		}
//	//	Serial.println("HIGH");
//
//	}
//
//	if (counter > 2000) {
//		counter = 0;
//		Serial.println("reset Counter");
//	}
}

void masterClockFunc(){
	float a = (micros()*1000)/50000000.0;
	uint32_t n = (sin(a)+1.0) * 32767.5;

	//uint32_t f = micros()/1000 % 65535;
	
	for (int i=0; i<8; i++){
		dac8718.setVoltage(i, n);
		//dac8718.setVoltage(i, 65536);
	}

}