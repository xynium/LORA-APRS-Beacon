#ifndef PINS_H_
#define PINS_H_

#undef OLED_SDA
#undef OLED_SCL
#undef OLED_RST

#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST 16

#define BUTTON_PIN GPIO_NUM_38 // The middle button GPIO 
#define RED_LED  GPIO_NUM_4 // red led on v1.1

#define GPS_RX 12
#define GPS_TX 34

#endif
