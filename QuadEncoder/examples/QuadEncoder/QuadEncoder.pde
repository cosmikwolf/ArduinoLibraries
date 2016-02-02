#include <QuadEncoder.h>
#include <Wire.h>
#include <LiquidTWI2.h>

int qe1Move=0;
int qeValue=0;
QuadEncoder qe(52,53);
LiquidTWI2 lcd(0x20);

void setup() {
  Serial.begin(9600);
  lcd.setMCPType(LTI_TYPE_MCP23017); 
  lcd.begin(16, 2);
  lcd.noAutoscroll();
  lcd.setBacklight(WHITE);
  lcd.setCursor(0, 0); 
  lcd.print("Basic Encoder Test:");

}

void loop() {
  qe1Move=qe.tick();
  if (qe1Move=='>') {
    Serial.print(char(qe1Move));
    qeValue += 1;
    lcd.setCursor(0,1);
    lcd.print("> " + String(qeValue)+ "     ");
  } else if (qe1Move=='<') {
    Serial.print(char(qe1Move));
    qeValue -= 1;
    lcd.setCursor(0,1);
    lcd.print("< " + String(qeValue)+ "     ");
  }
}