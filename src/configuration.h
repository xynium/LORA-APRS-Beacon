#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <iterator>
#include <list>

#include <Arduino.h>

class Configuration {
public:
  class Beacon {
  public:
    class Smart_Beacon {
    public:
      Smart_Beacon() : active(false), slow_rate(300), slow_speed(10), fast_rate(60), fast_speed(100) {
      }
      bool active;
      int  slow_rate;
      int  slow_speed;
      int  fast_rate;
      int  fast_speed;
    };

    Beacon() : callsign("NOCALL-7"), path("WIDE1-1"), message("LoRa Beacon"), timeout(1), symbol("["), overlay("/"), positiondilution(1) {
    }

    String       callsign;
    String       path;
    String       message;
    int          timeout;
    String       symbol;
    String       overlay;
    Smart_Beacon smart_beacon;
    int          positiondilution;
  };

  class LoRa {
  public:
    LoRa() : frequencyRx(433775000), frequencyTx(433775000), power(20), spreadingFactor(12), signalBandwidth(125000), codingRate4(5) {
    }

    long frequencyRx;
    long frequencyTx;
    int  power;
    int  spreadingFactor;
    long signalBandwidth;
    int  codingRate4;
  };

  class PTT {
  public:
    PTT() : active(false), io_pin(4), start_delay(0), end_delay(0), reverse(false) {
    }

    bool active;
    int  io_pin;
    int  start_delay;
    int  end_delay;
    bool reverse;
  };

  class Button {
  public:
    Button() : tx(false), alt_message(false) {
    }

    bool tx;
    int  alt_message;
  };

  Configuration() : debug(false) {
  }

  bool   debug;
  Beacon beacon;
  LoRa   lora;
  PTT    ptt;
  Button button;
};

class ConfigurationManagement {
public:
  explicit ConfigurationManagement(String FilePath);
  Configuration readConfiguration();

private:
  const String mFilePath;
};

#endif
