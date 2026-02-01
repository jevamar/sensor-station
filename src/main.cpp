#include <Esp.h>
#include <modules/ultrasonic_HC_SR04/HC_SR04.h>
#include <modules/BMx280/BMx280.h>
#include <Adafruit_SHT31.h>
// #include <WEMOS_SHT3X.h>
#include <utils.h>
#include <cfg.h>
#include <Wire.h>
#include <GyverINA.h>


// Вывод логов
#define IS_PRINT true
#define DEBUG_PRINTER Serial
#ifdef IS_PRINT
  #define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
  #define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
  #define DEBUG_PRINT(...) {}
  #define DEBUG_PRINTLN(...) {}
#endif


#ifndef LED_PIN
  // Значения по умолчанию если не определены через build_flags
  #ifdef ESP8266
    // #define LED_PIN D4
    // #define I2C_SDA D2
    // #define I2C_SCL D1
  #else
    // #define LED_PIN 2
    // #define I2C_SDA 6
    // #define I2C_SCL 7
  #endif
#endif

// Период опроса датчиков
uint32_t period = 1000;
uint32_t lastTime = 0;

// Пины для порта I2C
#define PIN_I2C_SDL I2C_SCL
#define PIN_I2C_SDA I2C_SDA

// Датчик температуры, давления и влажности
BMx280 bmx280;

// Датчик температуры и влажности SHT3x
Adafruit_SHT31 sht3x = Adafruit_SHT31();

// SHT3X sht3x(0x45);

// Ультразвуковой сенсор растояния
#define PIN_UKTRASONIC_TRIG TRIG_PIN
#define PIN_UKTRASONIC_ECHO ECHO_PIN
HC_SR04 sensorHcSr04(PIN_UKTRASONIC_TRIG, PIN_UKTRASONIC_ECHO);

// ВА-метр
INA219 ina; 

// Внешний сигнальный диод
// #define LED_PIN D1


void setup() 
{
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SDL);
  Serial.begin(115200);
  Serial.flush();

  delay(1000);

  DEBUG_PRINTLN(F(".........."));
  DEBUG_PRINTLN(F("Start init"));

  #ifdef ESP8266
    Serial.println("ESP8266 NodeMCU");
  #elif ESP32
    Serial.println("ESP32");
  #endif

  ledInit(LED_PIN);


  printChipInfo();

  cfg::chipId = 0 ; // ESP.getChipId();
  DEBUG_PRINT(F("Controller number: "));
  DEBUG_PRINTLN(cfg::chipId);
  
  sensorHcSr04.begin();
  
  if(bmx280.begin()){
    DEBUG_PRINT(F("Temp sensor: "));
    DEBUG_PRINTLN(bmx280.getName());
  } else {
    DEBUG_PRINTLN(F("Temp sensor not found"));
  }

  if(sht3x.begin()){
    DEBUG_PRINTLN(F("SHT3x sensor: OK"));
  } else {
    DEBUG_PRINTLN(F("SHT3x sensor not found"));
  }

  if (ina.begin()) {	// ina.begin(4, 5) // Для ESP32/ESP8266 можно указать пины I2C
    Serial.println(F("INA219 connected!"));
    Serial.print(F("Calibration value: ")); Serial.println(ina.getCalibration());
    ina.setResolution(INA219_VBUS, INA219_RES_12BIT_X4); 
    ina.setResolution(INA219_VSHUNT, INA219_RES_12BIT_X128);
  } else {
    Serial.println(F("INA219 not found!"));
  }


  DEBUG_PRINTLN(F("End init"));
}

void loopHcSr04()
{
  unsigned long resultUlrasonic = sensorHcSr04.readDistance();

  DEBUG_PRINT("Distance: ");
  DEBUG_PRINT(resultUlrasonic);
  DEBUG_PRINTLN(" cm");

  if (resultUlrasonic < 30) {
    ledOn(LED_PIN);
  } else {
    ledOff(LED_PIN);
  }
}

void loopBMx280()
{
  if(bmx280.read()) {
    DEBUG_PRINT("BMx280 Temp: ");
    DEBUG_PRINTLN(bmx280.readTemperature());

    DEBUG_PRINT("BMx280 Pressure: ");
    DEBUG_PRINTLN(bmx280.readPressure());

    DEBUG_PRINT("BMx280 Humidity: ");
    DEBUG_PRINTLN(bmx280.readHumidity());
  }
}


void loopSHT3x()
{
  auto t = sht3x.readTemperature();
	auto h = sht3x.readHumidity();

	if (isnan(h) || isnan(t))
	{
    return;
	}
  DEBUG_PRINT("SHT3x Temp: ");
  DEBUG_PRINTLN(t);

  DEBUG_PRINT("SHT3x Humidity: ");
  DEBUG_PRINTLN(h);
}

void loopINA219()
{
  // Читаем напряжение
  Serial.print(F("Voltage: "));
  Serial.print(ina.getVoltage(), 3);
  Serial.println(F(" V"));

  // Читаем ток
  Serial.print(F("Current: "));
  Serial.print(ina.getCurrent(), 3);
  Serial.println(F(" A"));

  // Читаем мощность
  Serial.print(F("Power: "));
  Serial.print(ina.getPower(), 3);
  Serial.println(F(" W"));

  // Читаем напряжение на шунте
  Serial.print(F("Shunt voltage: "));
  Serial.print(ina.getShuntVoltage(), 6);
  Serial.println(F(" V"));

  Serial.println("");
}

void loop()
{
  uint32_t currentTime = millis();
  if(currentTime - lastTime > period) {
    lastTime = currentTime;
    
    loopHcSr04();
    delay(10);

    loopBMx280();
    delay(10);
    
    loopSHT3x();
    delay(50);

    loopINA219();
    delay(50);
  }
}

/**
* Сканирование устройств на порте I2C
*/
void scannerI2C()
{
    byte error, address;
    int countDevices = 0;
 
    DEBUG_PRINTLN(F("Scanning..."));
 
    for(address = 8; address < 127; address++ ){
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
 
        if (error == 0){
            DEBUG_PRINT(F("I2C device found at address 0x"));
            if (address<16)
                DEBUG_PRINT("0");
            DEBUG_PRINT(address,HEX);
            DEBUG_PRINTLN(F(" !"));
 
            countDevices++;
        }
        else if (error==4) {
            DEBUG_PRINT(F("Unknow error at address 0x"));
            if (address<16) {
                DEBUG_PRINT(F("0"));
            }
            DEBUG_PRINTLN(address, HEX);
        } 
    }
    if (countDevices == 0) {
        DEBUG_PRINTLN(F("No I2C devices found\n"));
    } else {
        DEBUG_PRINTLN(F("done\n")); 
    } 
}

