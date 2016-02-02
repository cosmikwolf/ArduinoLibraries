
// Please read BounceMcp.h for information about the license and authors
#include <Wire.h>
#include "WProgram.h"
#include "BounceMcp.h"

BounceMcp::BounceMcp(Adafruit_MCP23017 mcpX, uint8_t pin,unsigned long interval_millis)
{
  interval(interval_millis);
  this->mcpX = mcpX;
  this->pin = pin;
}

void BounceMcp::begin(){
  previous_millis = millis();
  state = mcpX.digitalRead(pin);
}

void BounceMcp::write(int new_state)
       {
       	this->state = new_state;
       	mcpX.digitalWrite(pin,state);
       }


void BounceMcp::interval(unsigned long interval_millis)
{
  this->interval_millis = interval_millis;
  this->rebounce_millis = 0;
}

void BounceMcp::rebounce(unsigned long interval)
{
	 this->rebounce_millis = interval;
}



int BounceMcp::update()
{
	if ( debounce() ) {
        rebounce(0);
        return stateChanged = 1;
    }

     // We need to rebounce, so simulate a state change
     
	if ( rebounce_millis && (millis() - previous_millis >= rebounce_millis) ) {
        previous_millis = millis();
		 rebounce(0);
		 return stateChanged = 1;
	}

	return stateChanged = 0;
}


unsigned long BounceMcp::duration()
{
  return millis() - previous_millis;
}


int BounceMcp::read()
{
	return (int)state;
}


// Protected: debounces the pin
int BounceMcp::debounce() {
	
	uint8_t newState = mcpX.digitalRead(pin);
	if (state != newState ) {
  		if (millis() - previous_millis >= interval_millis) {
  			previous_millis = millis();
  			state = newState;
  			return 1;
	}
  }
  
  return 0;
	
}

// The risingEdge method is true for one scan after the de-bounced input goes from off-to-on.
bool  BounceMcp::risingEdge() { return stateChanged && state; }
// The fallingEdge  method it true for one scan after the de-bounced input goes from on-to-off. 
bool  BounceMcp::fallingEdge() { return stateChanged && !state; }

