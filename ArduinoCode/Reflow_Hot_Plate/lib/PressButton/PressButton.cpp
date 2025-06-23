#include "PressButton.h"


PressButton::PressButton(int pin){
    _IoPin = pin;
    pinMode(_IoPin,INPUT_PULLDOWN);
}


int PressButton::GetIoPin() {return _IoPin;}

boolean PressButton::IsDown(){return digitalRead(_IoPin) == HIGH && digitalRead(_IoPin) == HIGH;}
boolean PressButton::IsUp(){return digitalRead(_IoPin) == LOW && digitalRead(_IoPin) == LOW;}
boolean PressButton::CaptureDownState(){if (IsDown()) {WasDown = true;} return WasDown;}
boolean PressButton::ClearWasDown(){if (WasDown) {WasDown = false; return true;} return false;}
boolean PressButton::PressReleased(){if(WasDown && IsUp()){RepeatCnt = 0; WasDown = false; return true;} return false;}
boolean PressButton::longPressed(){return (Repeated() && RepeatCnt == 2);}
boolean PressButton::Repeated(){
    u_int32_t currMs = millis();

    if(WasDown && (
        RepeatCnt == 0 || 
        (RepeatCnt > 5 && currMs >= (LastRepeatMs + 200)) ||
        currMs >= (LastRepeatMs + (45 * (5 - RepeatCnt)))
    ))
    {
        if(RepeatCnt < 999){RepeatCnt += 1;}
        WasDown = false;
        LastRepeatMs = currMs;
        return true;
    }
    else{
        if(RepeatCnt > 0 && IsUp()){RepeatCnt = 0; WasDown = false;}
        return false;
    }
        
}