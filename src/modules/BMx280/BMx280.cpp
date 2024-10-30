/*
* Датчики температуры, давления BMP280 и влажности BME280
*/
#include "BMx280.h"

/**
* Инициализация датчика температуры, влажности и давления
*/
bool BMx280::begin()
{
    // BME280
    if (bme.begin(I2C_BMX280)) {  
        _isBME280 = true;
    }else{
        // BMP280
        if (bmp.begin(I2C_BMX280)) {  
            _isBMP280 = true;  
        }
    }
    return _isBMP280 || _isBME280;
}

bool BMx280::read(bool force)
{
    if(!_isBMP280 && !_isBME280) {
        return false;
    }
    uint32_t currentTime = millis();

    if(force || (currentTime - _lastReadTime > BMX280_MIN_INTERVAL)) {
        if(_isBMP280){
            _temp = bmp.readTemperature() + _tempCorrection;
            _pressure = bmp.readPressure() / 100.0f;
        }else if(_isBME280){
            _temp = bme.readTemperature() + _tempCorrection;
            _pressure = bme.readPressure() / 100.0f;
            _humidity = bme.readHumidity();
        }
        _lastReadTime = currentTime; 
    }
    return true;
}

String BMx280::getName()
{
    if(_isBME280) {
        return String("BME280");
    }
    if(_isBMP280) {
        return String("BMP280");
    }
    return String("");
}

// Получение данных с температурного датчика
float BMx280::readTemperature(bool force)
{
    read(force);
    return _temp;
}

float BMx280::readPressure(bool force)
{
    read(force);
    return _pressure;
}

float BMx280::readHumidity(bool force)
{
    read(force);
    return _humidity;
}

void BMx280::setTempCorrection(float correction)
{
    _tempCorrection = correction;
}
