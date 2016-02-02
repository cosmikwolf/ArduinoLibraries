/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#include <Wire.h>
#include <LiquidTWI2.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(5, 6);
//   avoid using pins with LEDs attached
LiquidTWI2 lcd(0x20);


#include <LiquidTWI2.h>

void setup() {
  Serial.begin(9600);
  lcd.setMCPType(LTI_TYPE_MCP23017); 
  lcd.begin(16, 2);
  lcd.noAutoscroll();
  lcd.setBacklight(WHITE);
  lcd.setCursor(0, 0); 
  lcd.print("Basic Encoder Test:");

}

long oldPosition  = -999;

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
  	lcd.setCursor(0,1);
  	lcd.print(String(newPosition)+ "     ");
  }
}
