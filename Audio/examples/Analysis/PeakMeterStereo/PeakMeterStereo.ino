/* Stereo peak meter example, assumes Audio adapter but just uses terminal so no more parts required.

This example code is in the public domain
*/

#include <Audio.h>
 #include <i2c_t3.h>
#include <SPI.h>
#include <SD.h>

//const int myInput = AUDIO_INPUT_LINEIN;
 const int myInput = AUDIO_INPUT_MIC;

AudioInputI2S        audioInput;         // audio shield: mic or line-in
AudioAnalyzePeak     peak_L;
AudioAnalyzePeak     peak_R;
AudioOutputI2S       audioOutput;        // audio shield: headphones & line-out
AudioFilterBiquad hp_filter;
AudioFilterBiquad lp_filter;
AudioFilterStateVariable state_filter;
AudioSynthWaveformDc filter_q;
AudioMixer4           mixer;

//AudioConnection c1(audioInput,0,peak_L,0);
//AudioConnection c2(audioInput,1,peak_R,0);
AudioConnection c1(lp_filter,0,peak_L,0);
AudioConnection c2(lp_filter,0,peak_R,0);

AudioConnection c3(audioInput,0, hp_filter,0);
AudioConnection c4(hp_filter,0, lp_filter,0);
//AudioConnection c3(audioInput,0, mixer,0);
//AudioConnection c4(audioInput,1, mixer,1);
//AudioConnection c5(mixer,0, bp_filter,0);
AudioConnection c6(filter_q,0, state_filter,1);
AudioConnection c7(lp_filter,0,audioOutput,0);
AudioConnection c8(lp_filter,0,audioOutput,1);
//AudioConnection c7(audioInput,1,audioOutput,0);
//AudioConnection c8(audioInput,1,audioOutput,1);

AudioControlSGTL5000 audioShield;


void setup() {
  AudioMemory(8);
  audioShield.enable();
  audioShield.inputSelect(myInput);
  audioShield.volume(0.7);
//  mixer.gain(0, 0.5);
// mixer.gain(1, 0.5);
 state_filter.frequency(500);
 state_filter.resonance(3);
 state_filter.octaveControl(0);
 filter_q.amplitude(1);
  hp_filter.setHighpass(0,500,0.7);
 // hp_filter.setHighpass(1,1000,0.7);
 // hp_filter.setHighpass(2,1000,0.7);
 // hp_filter.setHighpass(4,1000,0.7);
  lp_filter.setLowpass(0,2000,0.7);
 // lp_filter.setLowpass(1,5000,0.7);
 // lp_filter.setLowpass(2,5000,0.7);
 // lp_filter.setLowpass(4,5000,0.7);
  Serial.begin(9600);
}

// for best effect make your terminal/monitor a minimum of 62 chars wide and as high as you can.

elapsedMillis fps;
uint8_t cnt=0;

void loop() {

  if(fps > 24) {
    if (peak_L.available() && peak_R.available()) {
      fps=0;
      uint8_t leftPeak=peak_L.read() * 30.0;
      uint8_t rightPeak=peak_R.read() * 30.0;

      for(cnt=0;cnt<30-leftPeak;cnt++) {
        Serial.print(" ");
      }
      while(cnt++<30) {
        Serial.print("<");
      }
      Serial.print("||");
      for(cnt=0;cnt<rightPeak;cnt++) {
        Serial.print(">");
      }
      while(cnt++<30) {
        Serial.print(" ");
      }
      Serial.println();
    }
    Serial.println("memory: " + String(AudioMemoryUsageMax()));
  }
}
