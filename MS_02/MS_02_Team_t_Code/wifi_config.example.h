// wifi_config.example.h - copy this file to wifi_config.h and fill in your network.
//
// wifi_config.h is listed in .gitignore, so your password never goes to GitHub.
// A phone hotspot works fine: the Pico prints its address over USB serial.
// The network must be 2.4 GHz with WPA2 (the Pico W cannot join 5 GHz).
#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#define WIFI_SSID     "YOUR_WIFI_NAME"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

#endif // WIFI_CONFIG_H
