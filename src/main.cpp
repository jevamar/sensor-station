#include <Esp.h>
#include <modules/ultrasonic_HC_SR04/HC_SR04.h>
#include <modules/BMx280/BMx280.h>
#include <Adafruit_SHT31.h>
#include <SoftwareSerial.h>
// #include <WEMOS_SHT3X.h>
#include <utils.h>
#include <cfg.h>
#include <Wire.h>


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

constexpr const unsigned long SAMPLETIME_SDS_MS = 1000;   // Время между измерениями датчика загрязнености вовремя одного цикла измерения
constexpr const unsigned long WARMUPTIME_SDS_MS = 15000;	// Время разогрева датчика загрязнености воздуха, после которого можно снимать показания 
constexpr const unsigned long READINGTIME_SDS_MS = 5000;	// Время снятия показаний с датчика загрязненности воздуха
unsigned long act_milli;
unsigned long starttime;
unsigned long starttime_SDS;
bool is_SDS_running = true;
bool send_now = false;
String last_value_SDS_version;
unsigned long SDS_error_count;
#define msSince(timestamp_before) (act_milli - (timestamp_before))
#define UPDATE_MIN(MIN, SAMPLE) if (SAMPLE < MIN) { MIN = SAMPLE; }
#define UPDATE_MAX(MAX, SAMPLE) if (SAMPLE > MAX) { MAX = SAMPLE; }
#define UPDATE_MIN_MAX(MIN, MAX, SAMPLE) { UPDATE_MIN(MIN, SAMPLE); UPDATE_MAX(MAX, SAMPLE); }
bool is_SDS_running = true;
enum
{
	SDS_REPLY_HDR = 10,
	SDS_REPLY_BODY = 8
} SDS_waiting_for;
uint32_t sds_pm10_sum = 0;
uint32_t sds_pm25_sum = 0;
uint32_t sds_val_count = 0;
uint32_t sds_pm10_max = 0;
uint32_t sds_pm10_min = 20000;
uint32_t sds_pm25_max = 0;
uint32_t sds_pm25_min = 20000;
float last_value_SDS_P1 = -1.0;
float last_value_SDS_P2 = -1.0;
SoftwareSerial serialSDS;

// Период опроса датчиков
uint32_t period = 1000;
uint32_t lastTime = 0;
/** Конфиг: Интервал между измерениями станции */
unsigned sending_intervall_ms = 145000;

// Пины для порта I2C
#define PIN_I2C_SDL D5
#define PIN_I2C_SDA D6

// Датчик температуры, давления и влажности
BMx280 bmx280;

// Датчик температуры и влажности SHT3x
Adafruit_SHT31 sht3x = Adafruit_SHT31();

// SHT3X sht3x(0x45);

// Ультразвуковой сенсор растояния
#define PIN_UKTRASONIC_TRIG D2
#define PIN_UKTRASONIC_ECHO D3
HC_SR04 sensorHcSr04(PIN_UKTRASONIC_TRIG, PIN_UKTRASONIC_ECHO);

// Внешний сигнальный диод
#define LED_PIN D1


void setup() 
{
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SDL);
  Serial.begin(9600);
  Serial.flush();

  delay(1000);

  DEBUG_PRINTLN(F(".........."));
  DEBUG_PRINTLN(F("Start init"));

  ledInit(LED_PIN);


  cfg::chipId = ESP.getChipId();
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

	starttime = millis(); // store the start time
	starttime_SDS = starttime;

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
    String result_SDS;
    act_milli = millis();
    send_now = msSince(starttime) > sending_intervall_ms;
		if ((msSince(starttime_SDS) > SAMPLETIME_SDS_MS) || send_now)
		{
			starttime_SDS = act_milli;
      fetchSensorSDS(result_SDS);
    }
    starttime = millis(); // store the start time

    #if defined(ESP8266)
      // MDNS.update();
      serialSDS.perform_work();
    #endif
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

enum class PmSensorCmd {
	Start,
	Stop,
	ContinuousMode
};

bool SDS_checksum_valid(const uint8_t (&data)[8])
{
    uint8_t checksum_is = 0;
    for (unsigned i = 0; i < 6; ++i) {
        checksum_is += data[i];
    }
    return (data[7] == 0xAB && checksum_is == data[6]);
}

void SDS_rawcmd(const uint8_t cmd_head1, const uint8_t cmd_head2, const uint8_t cmd_head3)
{
	constexpr uint8_t cmd_len = 19;

	uint8_t buf[cmd_len];
	buf[0] = 0xAA;
	buf[1] = 0xB4;
	buf[2] = cmd_head1;
	buf[3] = cmd_head2;
	buf[4] = cmd_head3;
	for (unsigned i = 5; i < 15; ++i) {
		buf[i] = 0x00;
	}
	buf[15] = 0xFF;
	buf[16] = 0xFF;
	buf[17] = cmd_head1 + cmd_head2 + cmd_head3 - 2;
	buf[18] = 0xAB;
	serialSDS.write(buf, cmd_len);
}

bool SDS_cmd(PmSensorCmd cmd) 
{
	switch (cmd) {
	case PmSensorCmd::Start:
		SDS_rawcmd(0x06, 0x01, 0x01);
		break;
	case PmSensorCmd::Stop:
		SDS_rawcmd(0x06, 0x01, 0x00);
		break;
	case PmSensorCmd::ContinuousMode:
		// TODO: Check mode first before (re-)setting it
		SDS_rawcmd(0x08, 0x01, 0x00);
		SDS_rawcmd(0x02, 0x01, 0x00);
		break;
	}

	return cmd != PmSensorCmd::Stop;
}

/*****************************************************************
 * read SDS011 sensor serial and firmware date                   *
 *****************************************************************/
static String SDS_version_date()
{

	if (!last_value_SDS_version.length())
	{
		is_SDS_running = SDS_cmd(PmSensorCmd::Start);
		delay(250);
    #if defined(ESP8266)
		  serialSDS.perform_work();
    #endif
		serialSDS.flush();
		// Query Version/Date
		SDS_rawcmd(0x07, 0x00, 0x00);
		delay(400);
		const constexpr uint8_t header_cmd_response[2] = {0xAA, 0xC5};
		while (serialSDS.find(header_cmd_response, sizeof(header_cmd_response)))
		{
			uint8_t data[8];
			unsigned r = serialSDS.readBytes(data, sizeof(data));
			if (r == sizeof(data) && data[0] == 0x07 && SDS_checksum_valid(data))
			{
				char tmp[20];
				snprintf_P(tmp, sizeof(tmp), PSTR("%02d-%02d-%02d(%02x%02x)"),
						   data[1], data[2], data[3], data[4], data[5]);
				last_value_SDS_version = tmp;
				break;
			}
		}
	}

	return last_value_SDS_version;
}

static void sensor_restart()
{
  #if defined(ESP8266)
    // WiFi.disconnect();
    // WiFi.mode(WIFI_OFF);
    // delay(100);
  #endif

  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"

  // SPIFFS.end();

  #pragma GCC diagnostic pop

  serialSDS.end();
  delay(500);

  ESP.restart();
  // should not be reached
  while (true)
  {
    yield();
  }
}

/*****************************************************************
 * Измерение загрязненности воздуха датчика SDS011                                     *
 *****************************************************************/
static void fetchSensorSDS(String &s)
{
	if (sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS) &&
		msSince(starttime) < (sending_intervall_ms - (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
	{
    /**
     * Если конфиг интервала измерения станции больше чем время разогрева и чтения датчика загрязнения
     * и время последенго запуска измерения станции меньше заданого интервала измрения за вчетом времени разогрева и измерений
     */


    // Если датчик запущен, то останавливаем его
		if (is_SDS_running)
		{
			is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
		}
	}
	else
	{
    /**
     * Если после последенего измерения прошло больше времени интервала
     */

    // Если датчик не запущен, запускаем
		if (!is_SDS_running)
		{
			is_SDS_running = SDS_cmd(PmSensorCmd::Start);
			SDS_waiting_for = SDS_REPLY_HDR;
		}

		while (serialSDS.available() >= SDS_waiting_for)
		{
			const uint8_t constexpr hdr_measurement[2] = {0xAA, 0xC0};
			uint8_t data[8];

			switch (SDS_waiting_for)
			{
        case SDS_REPLY_HDR:
          if (serialSDS.find(hdr_measurement, sizeof(hdr_measurement)))
            SDS_waiting_for = SDS_REPLY_BODY;
          break;

        case SDS_REPLY_BODY:
          if (serialSDS.readBytes(data, sizeof(data)) == sizeof(data) && SDS_checksum_valid(data))
          {
            uint32_t pm25_serial = data[0] | (data[1] << 8);
            uint32_t pm10_serial = data[2] | (data[3] << 8);

            if (msSince(starttime) > (sending_intervall_ms - READINGTIME_SDS_MS))
            {
              sds_pm10_sum += pm10_serial;
              sds_pm25_sum += pm25_serial;
              UPDATE_MIN_MAX(sds_pm10_min, sds_pm10_max, pm10_serial);
              UPDATE_MIN_MAX(sds_pm25_min, sds_pm25_max, pm25_serial);

              DEBUG_PRINT(F("PM10 (sec.) : ")); 
              DEBUG_PRINTLN(String(pm10_serial / 10.0f));
              DEBUG_PRINT(F("PM2.5 (sec.): "));
              DEBUG_PRINTLN(String(pm25_serial / 10.0f));
              
              sds_val_count++;
            }
          }

          SDS_waiting_for = SDS_REPLY_HDR;
          break;
			}
		}
	}
	if (send_now)
	{
		last_value_SDS_P1 = -1;
		last_value_SDS_P2 = -1;

		if (sds_val_count > 2)
		{
			sds_pm10_sum = sds_pm10_sum - sds_pm10_min - sds_pm10_max;
			sds_pm25_sum = sds_pm25_sum - sds_pm25_min - sds_pm25_max;
			sds_val_count = sds_val_count - 2;
		}
		if (sds_val_count > 0)
		{
			last_value_SDS_P1 = float(sds_pm10_sum) / (sds_val_count * 10.0f);
			last_value_SDS_P2 = float(sds_pm25_sum) / (sds_val_count * 10.0f);

      
      // Вывод окончательных данных
      DEBUG_PRINT(F("PM10:  ")); 
      DEBUG_PRINTLN(last_value_SDS_P1);
      DEBUG_PRINT(F("PM2.5: "));
      DEBUG_PRINTLN(last_value_SDS_P2);


			if (sds_val_count < 3)
			{
        // Если измрений меньше 3, то увеличиваем счетчик ошибок
				SDS_error_count++;
			}
		}
		else
		{
      // Если измрений за цикл не было, то увеличиваем счетчик ошибок
			SDS_error_count++;
		}
		
    // обнуляем данные, после цикла измерений
    sds_pm10_sum = 0;
		sds_pm25_sum = 0;
		sds_val_count = 0;
		sds_pm10_max = 0;
		sds_pm10_min = 20000;
		sds_pm25_max = 0;
		sds_pm25_min = 20000;

    // Если заданный интервал больше веремени разогрева и измерений 
    // и датчик запущен, то отключаем его
		if ((sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
		{

			if (is_SDS_running)
			{
				is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
			}
		}
	}
}

