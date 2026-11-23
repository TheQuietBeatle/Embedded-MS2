// wifi_config.example.h - copy this file to wifi_config.h and fill in your network.
//
// wifi_config.h is listed in .gitignore, so your password never goes to GitHub.
// A phone hotspot works fine: the Pico prints its address over USB serial.
// The network must be 2.4 GHz (the Pico W cannot join 5 GHz).
//
// If the Pico can't join this network (or WIFI_SSID is ""), it starts its own
// Wi-Fi "NeuralCalibrator" (password robotarm123): join it and open
// http://192.168.4.1. To skip your router entirely, set WIFI_SSID to "".
#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#define WIFI_SSID     "YOUR_WIFI_NAME"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

#endif // WIFI_CONFIG_H
