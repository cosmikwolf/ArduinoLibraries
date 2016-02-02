/*  -------------------------------------------------------
    Zetaohm_SAM2695.cpp
    A library to facilitate playing music on the Modern Device Zetaohm_SAM2695 board.
    More info is at www.moderndevice.com; refer to "Using the Zetaohm_SAM2695 Arduino 
    Library" in the DocsWiki.
    -------------------------------------
    This version was derived from the library provided by Modern Devices
    . The NewSoftSerial library is used to communicate with the synth
    . The Constructor has been changed to take the rx and tx pin numbers
      (and instantiate the NewSoftSerial library); The default constructtor
      writes to pin 4, and disables receive.
    . A Polymorphic function, samWrite(...) has been added to write thru
      to the synth; it may be used alone, or in combination with existing
      library functions (use of samWrite must leave the synth capable of
      accepting library commands after, however).
    -------------------------------------
    This software is in the public domain.
    Modified 4/2011 R.McGinnis
    ------------------------------------------------------- */

#include "Arduino.h"
//#include "NewSoftSerial.h"
#include "PgmChange.h"
//#include "HardwareSerial.h"
#ifndef  Zetaohm_SAM2695_h
#define  Zetaohm_SAM2695_h

class Zetaohm_SAM2695
{
  private:
 // NewSoftSerial synth;
//    HardwareSerial * _HardSerial;
    byte synthInitialized;
    void begin();
  public:
    // default constructor sets NewSoftSerial to use pin 4 for tx, and inhibits rx
    Zetaohm_SAM2695();
    // constructor with 2 parameters sets NewSoftSerial's accordingly
 //   Zetaohm_SAM2695(byte rxPin, byte txPin);
  //  Zetaohm_SAM2695(HardwareSerial *serial);
    virtual size_t samWrite(byte c);
    virtual size_t samWrite(byte *buf, int cnt);
    void noteOn(byte channel, byte pitch, byte velocity);
    void noteOff(byte channel, byte pitch);
    void programChange (byte bank, byte channel, byte v);
    void pitchBend(byte channel, int v);
    void pitchBendRange(byte channel, byte v);
    void filterCutoff(byte channel, byte v);
    void filterResonance(byte channel, byte v);
    void midiReset();
    void setChannelVolume(byte channel, byte level);
    void setChannelBank(byte channel, byte bank);
	void allNotesOff(byte channel);
    void setMasterVolume(byte level);
    void setReverb(byte channel, byte program, byte level, byte delayFeedback);
    void setChorus(byte channel, byte program, byte level, byte feedback, byte chorusDelay);
};

#endif
