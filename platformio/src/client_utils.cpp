/* Client side utilities for esp32-weather-epd.
 * Copyright (C) 2022-2023  Luke Marzen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// built-in C++ libraries
#include <cstring>
#include <vector>

// arduino/esp32 libraries
#include <Arduino.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <time.h>
#include <WiFi.h>

// additional libraries
#include <Adafruit_BusIO_Register.h>
#include <ArduinoJson.h>

// header files
#include "_locale.h"
#include "api_response.h"
#include "aqi.h"
#include "client_utils.h"
#include "config.h"
#include "pass.h"
#include "display_utils.h"
#include "renderer.h"
#ifndef USE_HTTP
#include <WiFiClientSecure.h>
#endif

#ifdef USE_HTTP
static const uint16_t OWM_PORT = 80;
#else
static const uint16_t OWM_PORT = 443;
#endif

/* Power-on and connect WiFi.
 * Takes int parameter to store WiFi RSSI, or “Received Signal Strength
 * Indicator"
 *
 * Returns WiFi status.
 */
wl_status_t startWiFi(int &wifiRSSI)
{
  WiFi.mode(WIFI_STA);
  Serial.printf("Connecting to '%s'", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // timeout if WiFi does not connect in 10s from now
  unsigned long timeout = millis() + 10000;
  wl_status_t connection_status = WiFi.status();

  while ((connection_status != WL_CONNECTED) && (millis() < timeout))
  {
    Serial.print(".");
    delay(50);
    connection_status = WiFi.status();
  }
  Serial.println();

  if (connection_status == WL_CONNECTED)
  {
    wifiRSSI = WiFi.RSSI(); // get WiFi signal strength now, because the WiFi
                            // will be turned off to save power!
    Serial.println("IP: " + WiFi.localIP().toString());
  }
  else
  {
    Serial.printf("Could not connect to '%s'\n", WIFI_SSID);
  }
  return connection_status;
} // startWiFi

/* Disconnect and power-off WiFi.
 */
void killWiFi()
{
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}

/* Prints the local time to serial monitor.
 *
 * Returns true if getting local time was a success, otherwise false.
 */
bool printLocalTime(tm *timeInfo)
{
  int attempts = 0;
  while (!getLocalTime(timeInfo, 30000) && attempts++ < 5)
  {
    Serial.println("Failed to obtain time");
    return false;
  }
  Serial.println(timeInfo, "%A, %B %d, %Y %H:%M:%S");
  return true;
} // killWiFi

/* Connects to NTP server and stores time in a tm struct, adjusted for the time
 * zone specified in config.cpp.
 *
 * Returns true if success, otherwise false.
 *
 * Note: Must be connected to WiFi to get time from NTP server.
 */
bool setupTime(tm *timeInfo)
{
  // passing 0 for gmtOffset_sec and daylightOffset_sec and instead use setenv()
  // for timezone offsets
  configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
  setenv("TZ", TIMEZONE, 1);
  tzset();
  return printLocalTime(timeInfo);
} // setupTime
