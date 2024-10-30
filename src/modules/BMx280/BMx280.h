/*
* Датчики температуры, давления BMP280 и влажности BME280
*/
#ifndef BMX280_H
#define BMX280_H

#define I2C_BMX280 0x76 
#define BMX280_MIN_INTERVAL 1000 

#include "Esp.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>

class BMx280 {
    public:
        bool begin();
        bool read(bool force=false);
        float readTemperature(bool force=false);
        float readPressure(bool force=false);
        float readHumidity(bool force=false);
        void setTempCorrection(float correction);
        String getName();

    private:
        float _temp = 0;
        float _pressure = 0;
        float _humidity = 0;
        bool _isBMP280 = false;
        bool _isBME280 = false;
        float _tempCorrection = 0;
        uint32_t _lastReadTime = 0;
        Adafruit_BMP280 bmp; // I2C
        Adafruit_BME280 bme;
};

#endif //BMX280_H