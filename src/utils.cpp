/*
* Утилиты для управления переферией контроллера
*/
#include "utils.h"

void ledInit(uint8_t pin)
{
    pinMode(pin, OUTPUT);
}

void ledOn(uint8_t pin)
{
    digitalWrite(pin, HIGH);
}

void ledOff(uint8_t pin)
{
    digitalWrite(pin, LOW);
}
