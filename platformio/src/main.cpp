/* Main program for esp32-weather-epd.
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

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include <time.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Syslog.h>
#include <Wire.h>

#include "api_response.h"
#include "client_utils.h"
#include "config.h"
#include "display_utils.h"
#include "icons/icons_196x196.h"
#include "renderer.h"
#include "http_renderer.h"
#ifndef USE_HTTP
#include <WiFiClientSecure.h>
#endif
#ifdef USE_HTTPS_WITH_CERT_VERIF
#include "cert.h"
#endif

// too large to allocate locally on stack
static owm_resp_onecall_t owm_onecall;

Preferences prefs;

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udpClient;

// Syslog server connection info
#define SYSLOG_SERVER "pi.local"
#define SYSLOG_PORT 514

// This device info
#define DEVICE_HOSTNAME "esp32-frame"
#define APP_NAME "esp32-frame"

// Create a new syslog instance with LOG_KERN facility
Syslog syslog(udpClient, SYSLOG_SERVER, SYSLOG_PORT, DEVICE_HOSTNAME, APP_NAME, LOG_KERN);

/* Put esp32 into ultra low-power deep-sleep (<11μA).
 * Aligns wake time to the minute. Sleep times defined in config.cpp.
 */
void beginDeepSleep(unsigned long &startTime, tm *timeInfo)
{
  if (!getLocalTime(timeInfo))
  {
    Serial.println("Failed to obtain time before deep-sleep, referencing "
                   "older time.");
  }

  uint64_t sleepDuration = 0;
  int extraHoursUntilWake = 0;
  int curHour = timeInfo->tm_hour;

  if (timeInfo->tm_min >= 58)
  { // if we are within 2 minutes of the next hour, then round up for the
    // purposes of bed time
    curHour = (curHour + 1) % 24;
    extraHoursUntilWake += 1;
  }

  if (BED_TIME < WAKE_TIME && curHour >= BED_TIME && curHour < WAKE_TIME)
  { // 0              B   v  W  24
    // |--------------zzzzZzz---|
    extraHoursUntilWake += WAKE_TIME - curHour;
  }
  else if (BED_TIME > WAKE_TIME && curHour < WAKE_TIME)
  { // 0 v W               B    24
    // |zZz----------------zzzzz|
    extraHoursUntilWake += WAKE_TIME - curHour;
  }
  else if (BED_TIME > WAKE_TIME && curHour >= BED_TIME)
  { // 0   W               B  v 24
    // |zzz----------------zzzZz|
    extraHoursUntilWake += WAKE_TIME - (curHour - 24);
  }
  else // This feature is disabled (BED_TIME == WAKE_TIME)
  {    // OR it is not past BED_TIME
    extraHoursUntilWake = 0;
  }

  if (extraHoursUntilWake == 0)
  { // align wake time to nearest multiple of SLEEP_DURATION
    sleepDuration = SLEEP_DURATION * 60ULL - ((timeInfo->tm_min % SLEEP_DURATION) * 60ULL + timeInfo->tm_sec);
  }
  else
  { // align wake time to the hour
    sleepDuration = extraHoursUntilWake * 3600ULL - (timeInfo->tm_min * 60ULL + timeInfo->tm_sec);
  }

  // if we are within 2 minutes of the next alignment.
  if (sleepDuration <= 120ULL)
  {
    sleepDuration += SLEEP_DURATION * 60ULL;
  }

  // add extra delay to compensate for esp32's with fast RTCs.
  sleepDuration += 10ULL;

  esp_sleep_enable_timer_wakeup(sleepDuration * 1000000ULL);
  Serial.println("Awake for " + String((millis() - startTime) / 1000.0, 3) + "s");
  Serial.println("Deep-sleep for " + String(sleepDuration) + "s");
  esp_deep_sleep_start();
} // end beginDeepSleep

void writeToSyslog(const char *msg)
{
  Serial.println("Sending message to syslog...");
  syslog.log(LOG_INFO, msg);
  Serial.println("Syslog sent");
}

void sendSyslog()
{
  // Severity levels can be found in Syslog.h. They are same like in Linux
  // syslog.
  syslog.log(LOG_INFO, "Begin loop");

  // Log message can be formated like with printf function.
  syslog.logf(LOG_ERR, "This is error message no. %d", 1);
  syslog.logf(LOG_INFO, "This is info message no. %d", 2);

  // You can force set facility in pri parameter for this log message. More
  // facilities in syslog.h or in Linux syslog documentation.
  syslog.logf(LOG_DAEMON | LOG_INFO, "This is daemon info message no. %d",
              3);

  // F() macro is supported too
  syslog.log(LOG_INFO, F("End loop"));
}

/* Program entry point.
 */
void setup()
{
  unsigned long startTime = millis();
  Serial.begin(115200);

  String statusStr = {};
  String tmpStr = {};
  tm timeInfo = {};

  // START WIFI
  int wifiRSSI = 0; // “Received Signal Strength Indicator"
  wl_status_t wifiStatus = startWiFi(wifiRSSI);
  if (wifiStatus != WL_CONNECTED)
  { // WiFi Connection Failed
    killWiFi();
    if (wifiStatus == WL_NO_SSID_AVAIL)
    {
      Serial.println("SSID Not Available");
    }
    else
    {
      Serial.println("WiFi Connection Failed");
    }
    beginDeepSleep(startTime, &timeInfo);
  }

  // FETCH TIME
  bool timeConfigured = false;
  timeConfigured = setupTime(&timeInfo);
  if (!timeConfigured)
  { // Failed To Fetch The Time
    Serial.println("Failed To Fetch The Time");
    // killWiFi();
    // beginDeepSleep(startTime, &timeInfo);
  }
  String refreshTimeStr;
  getRefreshTimeStr(refreshTimeStr, timeConfigured, &timeInfo);

  sendSyslog();
  setupHttpRenderer(writeToSyslog);

  // DEEP-SLEEP
  beginDeepSleep(startTime, &timeInfo);
} // end setup

/* This will never run
 */
void loop()
{
} // end loop
