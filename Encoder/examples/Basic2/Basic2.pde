/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Wire.h>
#include <LiquidTWI2.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached
LiquidTWI2 lcd(0x20);


int pinA = 50; //Encoder pin A connects to interrupt 0 (D2)
int pinB = 51; //Encoder pin B connects to D4
int iValue = 0; //A variable that will be increased or decreased
                //when we turn the encoder


#include <LiquidTWI2.h>

void setup() {
  //Serial.begin(9600);
  lcd.setMCPType(LTI_TYPE_MCP23017); 
  lcd.begin(16, 2);
  lcd.noAutoscroll();
  lcd.setBacklight(WHITE);
  lcd.setCursor(0, 0); 
  lcd.print("Basic Encoder Test:");

  pinMode(pinA, INPUT);  
  pinMode(pinB, INPUT);
  attachInterrupt(50, encoderClick, RISING);

}


void loop() {

    delay(200);

}



void encoderClick(){
  // encoder must have turned one click because interrupt 0 was triggered
  // read value from both encoder pins
  int valA = digitalRead(pinA);
  int valB = digitalRead(pinB);
  
  // compare pins to determine in which direction encoder was turned
  if (valA != valB){
      // pinA just changed but pinB had not yet changed
      // Direction must be clockwise if A changes before B
      iValue++;
  }
  else{
      // pinA just changed and pinB had already done so.
      // Direction must be counter-clockwise if B changes before A
      iValue--;
  }
    lcd.setCursor(0,1);
    lcd.print(String(iValue)+ "     ");
}