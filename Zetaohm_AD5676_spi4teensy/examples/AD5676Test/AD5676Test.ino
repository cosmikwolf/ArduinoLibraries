#include <SPI.h>
#include <Zetaohm_AD5676.h>

Zetaohm_AD5676 ad5676;
elapsedMillis counter;


void setup(){
  Serial.begin(9600);
  Serial.println("Initializing AD5676 Octal DAC");
  ad5676.begin(15,14);
  ad5676.softwareReset();
  ad5676.internalReferenceEnable(true);
}

void loop(){

	int n = (sin(millis()/2.0)+1) / 2 * 65535;
	for (int i=0; i<8; i++){
		ad5676.setVoltage(n, i);
	}

/*	
  if (counter < 1000) {
    ad5676.setVoltage(65535, 0);
    ad5676.setVoltage(65535, 1);
    ad5676.setVoltage(65535, 2);
    ad5676.setVoltage(65535, 3);
    ad5676.setVoltage(65535, 4);
    ad5676.setVoltage(65535, 5);
    ad5676.setVoltage(65535, 6);
    ad5676.setVoltage(65535, 7);
  } else if (counter > 2000) {
    ad5676.setVoltage(0,  0);    
    ad5676.setVoltage(0,  1);    
    ad5676.setVoltage(0,  2);    
    ad5676.setVoltage(0,  3);    
    ad5676.setVoltage(0,  4);    
    ad5676.setVoltage(0,  5);    
    ad5676.setVoltage(0,  6);    
    ad5676.setVoltage(0,  7);    
    counter = 0;
      Serial.println("resetting...\t" + String(millis()));
  } else {
    ad5676.setVoltage(0,  0);    
    ad5676.setVoltage(0,  1);    
    ad5676.setVoltage(0,  2);    
    ad5676.setVoltage(0,  3);    
    ad5676.setVoltage(0,  4);    
    ad5676.setVoltage(0,  5);    
    ad5676.setVoltage(0,  6);    
    ad5676.setVoltage(0,  7);    
  }
  */
}

