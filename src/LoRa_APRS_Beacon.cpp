#include <Arduino.h>
#include <LoRa.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <WiFi.h>

#include "configuration.h"
#include "display.h"
#include "pins.h"
#include "power_management.h"
#include "sensor.h"

//#define Debug

// State of State machine
#define HasSynchGPS 1
#define PrepBeacon  2
#define Sleep       3

Configuration   mConfig;
PowerManagement powerManagement;
HardwareSerial  ss(1);
TinyGPSPlus     gps;
Adafruit_BMP280 bmp; // use I2C interface

void setup_gps();
void load_config();
void setup_lora();

String create_lat_aprs(RawDegrees lat);
String create_long_aprs(RawDegrees lng);
String createDateString(time_t t);
String createTimeString(time_t t);
String padding(unsigned int number, unsigned int width);

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL);
  powerManagement.begin(Wire);
  powerManagement.activateLoRa();
  powerManagement.activateOLED();
  powerManagement.activateGPS();
  powerManagement.activateMeasurement();
  WiFi.mode(WIFI_OFF);
  btStop();
  setup_display();
  load_config();
  setup_gps();
  setup_lora();
  powerManagement.clearCoulomb(); // Todo get when on usb stop charging for clear
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT_OPEN_DRAIN); // Led rouge io4 et Vcc
  digitalWrite(RED_LED, LOW);          // LedON
  gpio_wakeup_enable(BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_sleep_enable_timer_wakeup(mConfig.beacon.smart_beacon.slow_rate * 1000000);
  String sM = String("Beacon period: ") + String(mConfig.beacon.smart_beacon.slow_rate, DEC) + String("s");
  show_display("GO...", "", sM, "wait for position...", 1000);
}

void loop() {
  static unsigned int rate_limit_message_text = 0;
  String              batteryVoltage          = "";
  String              batteryChargeCurrent    = "";
  String              batteryCoulomb          = "";
  static int          iState;

  switch (iState) {
    case HasSynchGPS:
      {
        while (ss.available() > 0) {
          char c = ss.read();
          gps.encode(c);
        }
        if ((gps.time.isValid()) && (gps.time.age() < 2)) { // plus recent que 10s
          setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
          if (gps.location.isValid() && (gps.location.age() < 2)) {
#ifdef Debug
            Serial.println("GPS data ok");
#endif
            iState = PrepBeacon;
          }
        }
        if (powerManagement.isCharging()) {
          powerManagement.enableChgLed();
        } else {
          powerManagement.disableChgLed();
        }
        delay(10);
        break;
      }

    case PrepBeacon:
      {
        if (powerManagement.isBatteryConnect()) {
          batteryVoltage       = String(powerManagement.getBatteryVoltage(), 2) + "V";
          batteryChargeCurrent = String(powerManagement.getBatteryChargeDischargeCurrent(), 0) + "mA";
          batteryCoulomb       = String(powerManagement.getBatteryCoulomb(), 2) + "mAH";
        }
        char sFrame[255] = {
          '<',
          0xFF,
          0x01,
          0,
        };
        strcat(sFrame, mConfig.beacon.callsign.c_str());
        strcat(sFrame, ">");
        strcat(sFrame, "APLORA,"); // todo case path empty
        strcat(sFrame, mConfig.beacon.path.c_str());
        strcat(sFrame, ":!");
        strcat(sFrame, create_lat_aprs(gps.location.rawLat()).c_str());
        strcat(sFrame, mConfig.beacon.overlay.c_str());
        strcat(sFrame, create_long_aprs(gps.location.rawLng()).c_str());
        strcat(sFrame, mConfig.beacon.symbol.c_str());
        int course_int = max(0, min(360, (int)gps.course.deg()));
        if (course_int <= 0) {
          course_int += 360;
        }
        strcat(sFrame, padding(course_int, 3).c_str());
        strcat(sFrame, "/");
        int speed_int = max(0, min(999, (int)gps.speed.knots()));
        strcat(sFrame, padding(speed_int, 3).c_str());

        int alt_int = max(-99999, min(999999, (int)gps.altitude.feet()));
        if (alt_int < 0) {
          strcat(sFrame, "/A=-");
          strcat(sFrame, padding(alt_int * -1, 5).c_str());
        } else {
          strcat(sFrame, "/A=");
          strcat(sFrame, padding(alt_int, 6).c_str());
        }
        if (!(rate_limit_message_text++ % 4)) { // Comment rate one every N beacon
          strcat(sFrame, mConfig.beacon.message.c_str());
        }
        if (!powerManagement.isCharging()) {
          strcat(sFrame, "VBat= ");
          strcat(sFrame, batteryVoltage.c_str());
        }
        show_display(mConfig.beacon.callsign, createDateString(now()) + "   " + createTimeString(now()), String("Sats: ") + gps.satellites.value() + " HDOP: " + gps.hdop.hdop(), !powerManagement.isCharging() ? (String("Bat:") + batteryVoltage + ", " + batteryCoulomb) : "Powered via USB", 100);
        if (mConfig.ptt.active) {
          digitalWrite(mConfig.ptt.io_pin, mConfig.ptt.reverse ? LOW : HIGH);
          delay(mConfig.ptt.start_delay);
        } // fin formation Frame

        LoRa.beginPacket();
        LoRa.write((const uint8_t *)sFrame, strlen(sFrame));
        LoRa.endPacket();
        if (mConfig.ptt.active) {
          delay(mConfig.ptt.end_delay);
          digitalWrite(mConfig.ptt.io_pin, mConfig.ptt.reverse ? HIGH : LOW);
        }
        LoRa.sleep();
        iState = Sleep;
        break;
      }
    case Sleep:
      {
        powerManagement.deactivateGPS();
#ifdef Debug
        Serial.flush();
#endif
        digitalWrite(RED_LED, HIGH); // LedOFF
        esp_light_sleep_start();
        show_display("WOKE UP", "   ", "wait for position...", 100);
        powerManagement.activateGPS();
        Serial.println("awake");
        delay(500); // To get millis() change his value gps.time.age() on the old val
        iState = HasSynchGPS;
        break;
      }

    default:
      iState = HasSynchGPS;
      break;
  }
}

void load_config() {
  ConfigurationManagement confmg("/beacon.json");
  mConfig = confmg.readConfiguration();
  if (mConfig.beacon.callsign == "NOCALL-7") {
    show_display("ERROR", "You have to change your settings in 'data/tracker.json' and ", "upload it via \"Upload File System image\"!");
    while (true) {
    }
  }
}

void setup_lora() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  long freq = mConfig.lora.frequencyTx;
  if (!LoRa.begin(freq)) {
    while (true) {
    }
  }
  LoRa.setSpreadingFactor(mConfig.lora.spreadingFactor);
  LoRa.setSignalBandwidth(mConfig.lora.signalBandwidth);
  LoRa.setCodingRate4(mConfig.lora.codingRate4);
  LoRa.enableCrc();
  LoRa.setTxPower(mConfig.lora.power);
}

void setup_gps() {
  ss.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
}

char *s_min_nn(uint32_t min_nnnnn, int precision) {
  static char buf[8];
  double      dMin = min_nnnnn * 6e-8;
  sprintf(buf, "%07.4f", dMin);
  for (int it = (3 + precision); it < 5; it++)
    buf[it] = '0';
  buf[5] = 0;
  return buf;
}

String create_lat_aprs(RawDegrees lat) {
  char str[20];
  char n_s = 'N';
  if (lat.negative) {
    n_s = 'S';
  }
  sprintf(str, "%02d%s%c", lat.deg, s_min_nn(lat.billionths, mConfig.beacon.positiondilution), n_s);
  String lat_str(str);
  return lat_str;
}

String create_long_aprs(RawDegrees lng) {
  char str[20];
  char e_w = 'E';
  if (lng.negative) {
    e_w = 'W';
  }
  sprintf(str, "%03d%s%c", lng.deg, s_min_nn(lng.billionths, mConfig.beacon.positiondilution), e_w);
  String lng_str(str);
  return lng_str;
}

String createDateString(time_t t) {
  return String(padding(day(t), 2) + "." + padding(month(t), 2) + "." + padding(year(t), 4));
}

String createTimeString(time_t t) {
  return String(padding(hour(t), 2) + ":" + padding(minute(t), 2) + ":" + padding(second(t), 2));
}

String padding(unsigned int number, unsigned int width) {
  String result;
  String num(number);
  if (num.length() > width) {
    width = num.length();
  }
  for (unsigned int i = 0; i < width - num.length(); i++) {
    result.concat('0');
  }
  result.concat(num);
  return result;
}
