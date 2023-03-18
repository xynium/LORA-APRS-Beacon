# LoRa APRS Beacon

for TTGO T-Beam V1 (433MHz SX1278) fork from peterus repo.

The goal is increase battery duration,

left button reset
right button long press to stop
middle button force beacon

### How to compile

The best success is to use PlatformIO (and it is the only platform where I can support you). 

* Go to [PlatformIO](https://platformio.org/) download and install the IDE. 
* If installed open the IDE, go to the left side and klick on 'extensions' then search for 'PatformIO' and install.
* When installed click 'the ant head' on the left and choose import the project on the right.
* Just open the folder and you can compile the Firmware.

### Configuration

* You can find all nessesary settings to change for your configuration in **data/tracker.json**.
* To upload it to your board you have to do this via **Upload File System image** in PlatformIO!
* To find the 'Upload File System image' click the PlatformIO symbol (the little alien) on the left side, choos your configuration, click on 'Platform' and search for 'Upload File System image'.


