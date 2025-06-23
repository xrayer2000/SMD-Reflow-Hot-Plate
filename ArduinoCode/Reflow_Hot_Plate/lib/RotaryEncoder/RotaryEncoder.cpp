#include "RotaryEncoder.h"

RotaryEncoder::RotaryEncoder(int ENC_A, int ENC_B, int multiplier, int stepSize, int pauseLength) {
  _ENC_A = ENC_A;
  _ENC_B = ENC_B;
  _multiplier = multiplier;
  _stepSize = stepSize;
  _pauseLength = pauseLength;
  _oldENC_A = 0;
  _oldENC_B = 0;
  _lastENCread = 0;
  _ENCcounter = 0;
  _lastENCreadTime = micros();  
  pinMode(_ENC_A, INPUT_PULLUP);
  pinMode(_ENC_B, INPUT_PULLUP);
}
int RotaryEncoder::readEncoder() {
  int PinA = digitalRead(_ENC_A);
  int PinB = digitalRead(_ENC_B);
  int changevalue = 1;
  int turn = 0;

  if (PinA > _oldENC_A){     
    if (PinB != PinA) turn = -1; //clockWise 
    else turn = 1; //counterclockWise
    //Serial.print("turn: ");
    //Serial.println(turn);
  } 
  _oldENC_A = PinA;

  if(turn != 0) {
    if(turn == _lastENCread) {
        _ENCcounter++;
        //Serial.print("_ENCcounter: ");
        //Serial.println(_ENCcounter);
        //Serial.print("(micros() - _lastENCreadTime): ");
        //Serial.println((micros() - _lastENCreadTime));
        if((micros() - _lastENCreadTime) < _pauseLength) {
          changevalue = max((_ENCcounter/_stepSize)*_multiplier,1);
        }
        _lastENCreadTime = micros();
      } else {
        _ENCcounter=0;
    }
    _lastENCread = turn;
    //Serial.print("turn*changevalue: ");
  //Serial.println(turn*changevalue);
  }
  return turn*changevalue;
}
