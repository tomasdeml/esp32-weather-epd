// note that BMP bitmaps are drawn at physical position in physical orientation of the screen

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0
// uncomment next line to use class GFX of library GFX_Root instead of Adafruit_GFX
// #include <GFX.h>
// Note: if you use this with ENABLE_GxEPD2_GFX 1:
//       uncomment it in GxEPD2_GFX.h too, or add #include <GFX.h> before any #include <GxEPD2_GFX.h>

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <GxEPD2_7C.h>
#include <Syslog.h>
#include "config.h"

// NOTE: you may need to adapt or select for your wiring in the processor specific conditional compile sections below

// select the display class (only one), matching the kind of display panel
// #define GxEPD2_DISPLAY_CLASS GxEPD2_BW
#define GxEPD2_DISPLAY_CLASS GxEPD2_3C
// #define GxEPD2_DISPLAY_CLASS GxEPD2_7C

#define GxEPD2_DRIVER_CLASS GxEPD2_750c_Z08 // GDEW075Z08  800x480, EK79655 (GD7965), (WFT0583CZ61)

// somehow there should be an easier way to do this
#define GxEPD2_BW_IS_GxEPD2_BW true
#define GxEPD2_3C_IS_GxEPD2_3C true
#define GxEPD2_7C_IS_GxEPD2_7C true
#define GxEPD2_1248_IS_GxEPD2_1248 true
#define GxEPD2_1248c_IS_GxEPD2_1248c true
#define IS_GxEPD(c, x) (c##x)
#define IS_GxEPD2_BW(x) IS_GxEPD(GxEPD2_BW_IS_, x)
#define IS_GxEPD2_3C(x) IS_GxEPD(GxEPD2_3C_IS_, x)
#define IS_GxEPD2_7C(x) IS_GxEPD(GxEPD2_7C_IS_, x)
#define IS_GxEPD2_1248(x) IS_GxEPD(GxEPD2_1248_IS_, x)
#define IS_GxEPD2_1248c(x) IS_GxEPD(GxEPD2_1248c_IS_, x)

#define MAX_DISPLAY_BUFFER_SIZE 65536ul // e.g.
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8))

GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> httpDisplay(
    GxEPD2_DRIVER_CLASS(PIN_EPD_CS,
                        PIN_EPD_DC,
                        PIN_EPD_RST,
                        PIN_EPD_BUSY));

#include <WiFiClient.h>
#include <WiFiClientSecure.h>

bool showBitmapFrom_HTTP(Logger log, const char *host, int port, const char *path, const char *filename, int16_t x, int16_t y, bool with_color = true);

void setupHttpRenderer(Logger syslogger, Logger log)
{
  log(LOG_DEBUG, "Starting HTTP renderer");

  httpDisplay.epd2.setLogger(syslogger);
  if (!showBitmapFrom_HTTP(log, "10.11.1.2", 8080, "/", "ping", 0, 0, true))
  {
    // TODO Only if powered on
    httpDisplay.powerOff();
  }

  log(LOG_DEBUG, "Completed HTTP renderer");
}

uint16_t read16(WiFiClient &client)
{
  // BMP data is stored little-endian, same as Arduino.
  uint16_t result;
  ((uint8_t *)&result)[0] = client.read(); // LSB
  ((uint8_t *)&result)[1] = client.read(); // MSB
  return result;
}

uint32_t read32(WiFiClient &client)
{
  // BMP data is stored little-endian, same as Arduino.
  uint32_t result;
  ((uint8_t *)&result)[0] = client.read(); // LSB
  ((uint8_t *)&result)[1] = client.read();
  ((uint8_t *)&result)[2] = client.read();
  ((uint8_t *)&result)[3] = client.read(); // MSB
  return result;
}

uint32_t skip(WiFiClient &client, int32_t bytes)
{
  int32_t remain = bytes;
  uint32_t start = millis();
  while ((client.connected() || client.available()) && (remain > 0))
  {
    if (client.available())
    {
      client.read();
      remain--;
    }
    else
      delay(1);
    if (millis() - start > 2000)
      break; // don't hang forever
  }
  return bytes - remain;
}

uint32_t read8n(WiFiClient &client, uint8_t *buffer, int32_t bytes)
{
  // uint32_t start = millis();
  int32_t read = client.read(buffer, bytes);
  if (read != bytes)
  {
    delay(1);
    bool connected = client.connected();
    bool avaialble = client.available();
    Serial.printf("read = %d, conn = %d, avail = %d\n", read, connected, avaialble);
  }
  // Serial.printf("read of %d/%d bytes took %3lu ms\n", read, bytes, (millis() - start));

  return read > 0 ? read : 0;
}

static const uint16_t input_buffer_pixels = 800; // may affect performance

static const uint16_t max_row_width = 1872;     // for up to 7.8" display 1872x1404
static const uint16_t max_palette_pixels = 256; // for depth <= 8

uint8_t input_buffer[3 * input_buffer_pixels];        // up to depth 24
uint8_t output_row_mono_buffer[max_row_width / 8];    // buffer for at least one row of b/w bits
uint8_t output_row_color_buffer[max_row_width / 8];   // buffer for at least one row of color bits
uint8_t mono_palette_buffer[max_palette_pixels / 8];  // palette buffer for depth <= 8 b/w
uint8_t color_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 c/w
uint16_t rgb_palette_buffer[max_palette_pixels];      // palette buffer for depth <= 8 for buffered graphics, needed for 7-color display

void serialPrint(const char *message)
{
  unsigned long currentMicros = micros();
  unsigned long seconds = currentMicros / 1000000;
  unsigned long milliseconds = (currentMicros % 1000000) / 1000;

  char timestamp[10];

  snprintf(timestamp, sizeof(timestamp), "%02lu.%03lu",
           seconds, milliseconds);

  Serial.printf("[%s] %s", timestamp, message);
}

bool showBitmapFrom_HTTP(Logger log, const char *host, int port, const char *path, const char *filename, int16_t x, int16_t y, bool with_color)
{
  WiFiClient client;
  client.setTimeout(120);

  bool connection_ok = false;
  bool valid = false; // valid format to be handled
  bool flip = true;   // bitmap is stored bottom-to-top

  uint32_t startTime = millis();
  if ((x >= httpDisplay.epd2.WIDTH) || (y >= httpDisplay.epd2.HEIGHT))
    return false;

  Serial.println();
  Serial.print("downloading file \"");
  Serial.print(filename);
  Serial.println("\"");
  Serial.print("connecting to ");
  Serial.println(host);
  if (!client.connect(host, port))
  {
    log(LOG_ERR, "connection failed");
    return false;
  }

  serialPrint("requesting URL: ");
  Serial.println(String("http://") + host + path + filename);
  client.print(String("GET ") + path + filename + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: GxEPD2_WiFi_Example\r\n" +
               "X-ESP32-BED-TIME: " + BED_TIME + "\r\n" +
               "X-ESP32-WAKE-TIME: " + WAKE_TIME + "\r\n" +
               "Connection: close\r\n\r\n");
  log(LOG_DEBUG, "request sent");
  while (client.connected())
  {
    String line = client.readStringUntil('\n');

    if (!connection_ok)
    {
      connection_ok = line.startsWith("HTTP/1.1 200 OK");
      // if (connection_ok)
      //   Serial.println(line);
    }
    // if (!connection_ok)
    //   Serial.println(line);

    if (line == "\r")
    {
      serialPrint("header received\n");
      log(LOG_DEBUG, "headers received");
      break;
    }
  }
  if (!connection_ok)
    return false;

  serialPrint("init\n");
  httpDisplay.init(115200, true, 2, false);
  SPI.begin(PIN_EPD_SCK,
            PIN_EPD_MISO,
            PIN_EPD_MOSI,
            PIN_EPD_CS);
  serialPrint("init completed\n");

  // Parse BMP header
  uint16_t sig = read16(client);
  Serial.print("BMP signature: ");
  Serial.println(sig);

  if (sig == 0xC0FF)
  {
    log(LOG_INFO, "Clearing display...");
    httpDisplay.clearScreen();
    httpDisplay.powerOff();
    valid = true;
  }
  else if (sig == 0x4D42) // BMP signature
  {
    serialPrint("BMP signature matched\n");
    log(LOG_INFO, "BMP signature matched");

    uint32_t fileSize = read32(client);
    uint32_t creatorBytes = read32(client);
    (void)creatorBytes;                    // unused
    uint32_t imageOffset = read32(client); // Start of image data
    uint32_t headerSize = read32(client);
    uint32_t width = read32(client);
    int32_t height = (int32_t)read32(client);
    uint16_t planes = read16(client);
    uint16_t depth = read16(client); // bits per pixel
    uint32_t format = read32(client);
    uint32_t bytes_read = 7 * 4 + 3 * 2;                   // read so far
    if ((planes == 1) && ((format == 0) || (format == 3))) // uncompressed is handled, 565 also
    {
      Serial.print("File size: ");
      Serial.println(fileSize);
      Serial.print("Image Offset: ");
      Serial.println(imageOffset);
      Serial.print("Header size: ");
      Serial.println(headerSize);
      Serial.print("Bit Depth: ");
      Serial.println(depth);
      Serial.print("Image size: ");
      Serial.print(width);
      Serial.print('x');
      Serial.println(height);
      // BMP rows are padded (if needed) to 4-byte boundary
      uint32_t rowSize = (width * depth / 8 + 3) & ~3;
      if (depth < 8)
        rowSize = ((width * depth + 8 - depth) / 8 + 3) & ~3;
      if (height < 0)
      {
        height = -height;
        flip = false;
      }
      uint16_t w = width;
      uint16_t h = height;
      if ((x + w - 1) >= httpDisplay.epd2.WIDTH)
        w = httpDisplay.epd2.WIDTH - x;
      if ((y + h - 1) >= httpDisplay.epd2.HEIGHT)
        h = httpDisplay.epd2.HEIGHT - y;
      if (w <= max_row_width) // handle with direct drawing
      {
        valid = true;
        uint8_t bitmask = 0xFF;
        uint8_t bitshift = 8 - depth;
        uint16_t red, green, blue;
        bool whitish = false;
        bool colored = false;
        if (depth == 1)
          with_color = false;
        if (depth <= 8)
        {
          if (depth < 8)
            bitmask >>= depth;
          // bytes_read += skip(client, 54 - bytes_read); //palette is always @ 54
          bytes_read += skip(client, imageOffset - (4 << depth) - bytes_read); // 54 for regular, diff for colorsimportant
          for (uint16_t pn = 0; pn < (1 << depth); pn++)
          {
            blue = client.read();
            green = client.read();
            red = client.read();
            client.read();
            bytes_read += 4;
            whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
            colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0));                                                  // reddish or yellowish?
            if (0 == pn % 8)
              mono_palette_buffer[pn / 8] = 0;
            mono_palette_buffer[pn / 8] |= whitish << pn % 8;
            if (0 == pn % 8)
              color_palette_buffer[pn / 8] = 0;
            color_palette_buffer[pn / 8] |= colored << pn % 8;
          }
        }
        serialPrint("pallete read\n");
        httpDisplay.clearScreen();
        serialPrint("starting rendering\n");
        uint32_t rowPosition = flip ? imageOffset + (height - h) * rowSize : imageOffset;
        // Serial.print("skip "); Serial.println(rowPosition - bytes_read);
        bytes_read += skip(client, rowPosition - bytes_read);
        for (uint16_t row = 0; row < h; row++, rowPosition += rowSize) // for each line
        {
          if (!connection_ok || !(client.connected() || client.available()))
          {
            log(LOG_ERR, (String("read loop terminated at row ") + row).c_str());
            break;
          }
          delay(1); // yield() to avoid WDT
          // serialPrint("reading row data\n");
          uint32_t in_remain = rowSize;
          uint32_t in_idx = 0;
          uint32_t in_bytes = 0;
          uint8_t in_byte = 0;           // for depth <= 8
          uint8_t in_bits = 0;           // for depth <= 8
          uint8_t out_byte = 0xFF;       // white (for w%8!=0 border)
          uint8_t out_color_byte = 0xFF; // white (for w%8!=0 border)
          uint32_t out_idx = 0;
          for (uint16_t col = 0; col < w; col++) // for each pixel
          {
            yield();
            // SLOW DOWN HERE WHEN CALLING INTO CLIENT
            // But without it read8n() fails to read the whole buffer and available() becomes 0
            if (!connection_ok || !(client.connected() || client.available()))
            {
              // log(LOG_DEBUG, (String("read loop terminated at column ") + col + " of row " + row).c_str());
              break;
            }
            // Time to read more pixel data?
            if (in_idx >= in_bytes) // ok, exact match for 24bit also (size IS multiple of 3)
            {
              uint32_t get = in_remain > sizeof(input_buffer) ? sizeof(input_buffer) : in_remain;
              uint32_t got = read8n(client, input_buffer, get);
              while ((got < get) && connection_ok)
              {
                // Serial.print("got "); Serial.print(got); Serial.print(" < "); Serial.print(get); Serial.print(" @ "); Serial.println(bytes_read);
                uint32_t gotmore = read8n(client, input_buffer + got, get - got);
                got += gotmore;
                connection_ok = gotmore > 0;
              }
              in_bytes = got;
              in_remain -= got;
              bytes_read += got;
            }
            if (!connection_ok)
            {
              Serial.print("Error: got no more after ");
              Serial.print(bytes_read);
              Serial.println(" bytes read!");
              log(LOG_ERR, (String("go no more after ") + bytes_read + " bytes read").c_str());
              break;
            }
            switch (depth)
            {
            case 32:
              blue = input_buffer[in_idx++];
              green = input_buffer[in_idx++];
              red = input_buffer[in_idx++];
              in_idx++;                                                                                                     // skip alpha
              whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
              colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0));                                                  // reddish or yellowish?
              break;
            case 24:
              blue = input_buffer[in_idx++];
              green = input_buffer[in_idx++];
              red = input_buffer[in_idx++];
              whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
              colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0));                                                  // reddish or yellowish?
              break;
            case 16:
            {
              uint8_t lsb = input_buffer[in_idx++];
              uint8_t msb = input_buffer[in_idx++];
              if (format == 0) // 555
              {
                blue = (lsb & 0x1F) << 3;
                green = ((msb & 0x03) << 6) | ((lsb & 0xE0) >> 2);
                red = (msb & 0x7C) << 1;
              }
              else // 565
              {
                blue = (lsb & 0x1F) << 3;
                green = ((msb & 0x07) << 5) | ((lsb & 0xE0) >> 3);
                red = (msb & 0xF8);
              }
              whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
              colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0));                                                  // reddish or yellowish?
            }
            break;
            case 1:
            case 2:
            case 4:
            case 8:
            {
              if (0 == in_bits)
              {
                in_byte = input_buffer[in_idx++];
                in_bits = 8;
              }
              uint16_t pn = (in_byte >> bitshift) & bitmask;
              whitish = mono_palette_buffer[pn / 8] & (0x1 << pn % 8);
              colored = color_palette_buffer[pn / 8] & (0x1 << pn % 8);
              in_byte <<= depth;
              in_bits -= depth;
            }
            break;
            }
            if (whitish)
            {
              // keep white
            }
            else if (colored && with_color)
            {
              out_color_byte &= ~(0x80 >> col % 8); // colored
            }
            else
            {
              out_byte &= ~(0x80 >> col % 8); // black
            }
            if ((7 == col % 8) || (col == w - 1)) // write that last byte! (for w%8!=0 border)
            {
              output_row_color_buffer[out_idx] = out_color_byte;
              output_row_mono_buffer[out_idx++] = out_byte;
              out_byte = 0xFF;       // white (for w%8!=0 border)
              out_color_byte = 0xFF; // white (for w%8!=0 border)
            }
          } // end pixel
          int16_t yrow = y + (flip ? h - row - 1 : row);
          // serialPrint("row processed "); Serial.println(row);
          httpDisplay.writeImage(output_row_mono_buffer, output_row_color_buffer, x, yrow, w, 1);
          // serialPrint("row written\n");
        } // end line
        serialPrint("downloaded in ");
        Serial.print(millis() - startTime);
        Serial.println(" ms");
        httpDisplay.refresh(false);
      }
      serialPrint("bytes read ");
      Serial.println(bytes_read);
    }
  }
  client.stop();

  if (!valid)
  {
    Serial.println("Bitmap format not handled");
  }

  return valid;
}
