/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME  "AyleenRV"
#define IO_KEY       "aio_zmCd92gGD1kdJxjAiFM0ofIkDrXl"

/******************************* WIFI **************************************/

#define WIFI_SSID "Ayleen's Galaxy A32"
#define WIFI_PASS "kzpf3227"

// comment out the following lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"


AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

