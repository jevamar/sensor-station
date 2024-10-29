#include <Esp.h>
#include <modules\ultrasonic_HC_SR04\HC_SR04.h>
#include <utils.h>

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

// Ультразвуковой сенсор растояния
#define PIN_UKTRASONIC_TRIG D2
#define PIN_UKTRASONIC_ECHO D3
HC_SR04 sensorHcSr04(PIN_UKTRASONIC_TRIG, PIN_UKTRASONIC_ECHO);

// Внешний сигнальный диод
#define LED_PIN D1


void setup() 
{
  DEBUG_PRINTLN("Start init");
  
  ledInit(LED_PIN);
  Serial.begin(9600);
  sensorHcSr04.begin();

  DEBUG_PRINTLN("End init");
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

void loop()
{
  loopHcSr04();
  
  delay(200);
}