/*
* Ультразвуковой сенсор расстояния HC-SR04
*/
#ifndef HC_SR04_H
#define HC_SR04_H

#include "Esp.h"

class HC_SR04 {
    public:
        HC_SR04(uint8_t pinTrig, uint8_t pinEcho);
        bool begin(void);
        bool read(bool force=false);
        unsigned long readDistance(bool force=false);
        // TODO: getName

    private:
        unsigned long _distance;
        uint8_t _pinTrig, _pinEcho;
};

#endif //HC_SR04_H
