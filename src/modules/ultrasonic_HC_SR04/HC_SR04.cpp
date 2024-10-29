/*
* Ультразвуковой сенсор расстояния HC-SR04
*/
#include "HC_SR04.h"

HC_SR04::HC_SR04(uint8_t pinTrig, uint8_t pinEcho)
{
    _pinTrig = pinTrig;
    _pinEcho = pinEcho;
}

bool HC_SR04::begin(void)
{
    pinMode(_pinTrig, OUTPUT);
    return true;
}

bool HC_SR04::read(bool force)
{
    // Создаем короткий импульс длительностью 5 микросекунд.
    digitalWrite(_pinTrig, LOW);
    delayMicroseconds(5);
    digitalWrite(_pinTrig, HIGH);
    // Установим высокий уровень сигнала
    delayMicroseconds(10);
    digitalWrite(_pinTrig, LOW);
    //  Определяем задержку сигнала
    unsigned long duration = pulseIn(_pinEcho, HIGH);
    // Преобразуем время задержки в расстояние
    _distance = (duration / 2) / 29.1;

    return true;
}

unsigned long HC_SR04::readDistance(bool force)
{
    read(force);
    return _distance;
}
