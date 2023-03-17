#include <APRS-Decoder.h>
#include <Arduino.h>
#include <LoRa.h>
#include <OneButton.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <WiFi.h>

#include "configuration.h"
#include "display.h"
#include "pins.h"
#include "power_management.h"

// State of State machine
#define HasSynchGPS 1
#define PrepBeacon  2
#define Sleep       3

Configuration   mConfig;
PowerManagement powerManagement;
OneButton       userButton = OneButton(BUTTON_PIN, true, true);
HardwareSerial  ss(1);
TinyGPSPlus     gps;

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
  // make sure wifi and bt is off as we don't need it:
  WiFi.mode(WIFI_OFF);
  btStop();
  setup_display();
  load_config();
  setup_gps();
  setup_lora();
  powerManagement.clearCoulomb(); // Todo get when on usb stop charging for clear
  // userButton.attachClick(handle_click); // todo
  //  userButton.attachLongPressStart(handle_longPress);
  //  userButton.attachDoubleClick(handle_dblClk);
  esp_sleep_enable_timer_wakeup(mConfig.beacon.smart_beacon.slow_rate * 1000000);
  String sM = String("Beacon period: ") + String(mConfig.beacon.smart_beacon.slow_rate, DEC) + String("s");
  show_display("GO...", "", sM, "wait for position...", 1000);
}

void loop() {
  static unsigned int rate_limit_message_text = 0;
  String              batteryVoltage          = "";
  String              batteryChargeCurrent    = "";
  String              batteryCoulomb          = "";
  APRSMessage         msg;
  String              lat;
  String              lng;
  String              aprsmsg;
  static int          iState;

  // userButton.tick();  TODO
  switch (iState) {
  case HasSynchGPS: {
    while (ss.available() > 0) {
      char c = ss.read();
      gps.encode(c);
    }
    if (gps.time.isValid()) {
      setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
      // Serial.println("GPS time  ok");
    }
    if (gps.location.isValid()) {
      Serial.println("GPS pos ok");
      iState = PrepBeacon;
    }
    if (powerManagement.isCharging()) {
      powerManagement.enableChgLed();
    } else {
      powerManagement.disableChgLed();
    }
    delay(2);
    break;
  }

  case PrepBeacon: {
    if (powerManagement.isBatteryConnect()) {
      batteryVoltage       = String(powerManagement.getBatteryVoltage(), 2) + "V";
      batteryChargeCurrent = String(powerManagement.getBatteryChargeDischargeCurrent(), 0) + "mA";
      batteryCoulomb       = String(powerManagement.getBatteryCoulomb(), 2) + "mAH";
    }
    msg.setSource(mConfig.beacon.callsign);
    msg.setPath(mConfig.beacon.path);
    msg.setDestination("APLORA");
    lat            = create_lat_aprs(gps.location.rawLat());
    lng            = create_long_aprs(gps.location.rawLng());
    String alt     = "";
    int    alt_int = max(-99999, min(999999, (int)gps.altitude.feet()));
    if (alt_int < 0) {
      alt = "/A=-" + padding(alt_int * -1, 5);
    } else {
      alt = "/A=" + padding(alt_int, 6);
    }
    int    speed_int  = max(0, min(999, (int)gps.speed.knots()));
    String speed      = padding(speed_int, 3);
    int    course_int = max(0, min(360, (int)gps.course.deg()));
    /* course in between 1..360 due to aprs spec */
    if (course_int == 0) {
      course_int = 360;
    }
    String course = padding(course_int, 3);
    aprsmsg       = "!" + lat + mConfig.beacon.overlay + lng + mConfig.beacon.symbol + course + "/" + speed + alt;
    if (!(rate_limit_message_text++ % 4)) { // Comment rate one every N beacon
      aprsmsg += mConfig.beacon.message;
    }
    if (!powerManagement.isCharging()) {
      aprsmsg += "VBat= " + batteryVoltage + "Cur= " + batteryChargeCurrent;
    }
    msg.getBody()->setData(aprsmsg);
    String data = msg.encode();
    // Serial.println("TX ok");
    show_display(mConfig.beacon.callsign, createDateString(now()) + "   " + createTimeString(now()), String("Sats: ") + gps.satellites.value() + " HDOP: " + gps.hdop.hdop(), !powerManagement.isCharging() ? (String("Bat:") + batteryVoltage + ", " + batteryCoulomb) : "Powered via USB", 100);
    if (mConfig.ptt.active) {
      digitalWrite(mConfig.ptt.io_pin, mConfig.ptt.reverse ? LOW : HIGH);
      delay(mConfig.ptt.start_delay);
    }
    LoRa.beginPacket();
    LoRa.write('<'); // Header
    LoRa.write(0xFF);
    LoRa.write(0x01);
    LoRa.write((const uint8_t *)data.c_str(), data.length());
    LoRa.endPacket();
    if (mConfig.ptt.active) {
      delay(mConfig.ptt.end_delay);
      digitalWrite(mConfig.ptt.io_pin, mConfig.ptt.reverse ? HIGH : LOW);
    }
    iState = Sleep;
    break;
  }
  case Sleep: {
    powerManagement.deactivateGPS();
    // Serial.flush();
    esp_light_sleep_start();
    // Serial.println("wake up");
    show_display("WOKE UP", "   ", "wait for position...", 100);
    powerManagement.activateGPS();
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
  double dMin = min_nnnnn * 6e-8;
  sprintf(buf, "%07.4f", dMin);
  buf[3 + precision] = 0;
  Serial.println(buf);
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
