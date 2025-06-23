#include <Arduino.h>

class PressButton
{
    private:
        int _IoPin;
    public:
        PressButton(int pin);
        int GetIoPin();
        boolean WasDown = false;
        uint32_t RepeatCnt = 0;
        uint32_t LastRepeatMs = 0;
        boolean IsDown();
        boolean IsUp();
        boolean CaptureDownState();
        boolean ClearWasDown();
        boolean PressReleased();
        boolean longPressed();
        boolean Repeated();
};
