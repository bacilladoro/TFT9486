#include <Arduino.h>
#define LCD_CS PB6 // Chip Select goes to Analog 3
#define LCD_CD PB5 // Command/Data goes to Analog 2
#define LCD_WR PB4 // LCD Write goes to Analog 1
#define LCD_RD PB3 // LCD Read goes to Analog 0
#define LCD_RESET PB7 // Can alternately just connect to Arduino's reset pin


#include <FreeDefaultFonts.h>
#define ADJ_BASELINE 0 

#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Fruit_GFX.h"
#include <mcutft.h>

mcutft tft;
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define RGB(r, g, b) (((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3))

#define GREY      RGB(127, 127, 127)
#define DARKGREY  RGB(64, 64, 64)
#define TURQUOISE RGB(0, 128, 128)
#define PINK      RGB(255, 128, 192)
#define OLIVE     RGB(128, 128, 0)
#define PURPLE    RGB(128, 0, 128)
#define AZURE     RGB(0, 128, 255)
#define ORANGE    RGB(255,128,64)
 
#include <stdio.h>

uint16_t ID;
uint8_t hh, mm, ss; //containers for current time
int16_t x, y, dx, dy, radius = 160, idx;
uint16_t w, h, len, mask;
uint16_t colors[8] = { BLACK, WHITE, YELLOW, CYAN, GREEN, MAGENTA, RED, BLUE };
int height, width;

uint8_t conv2d(const char* p)
{
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9') v = *p - '0';
    return 10 * v + *++p - '0';
}

void setup(void)
{
    Serial.begin(9600);
    tft.reset();
    ID = tft.readID();
    Serial.print("TFT ID = 0x");
    Serial.println(ID, HEX);
    //    if (ID == 0xD3D3) ID = 0x9481; // write-only shield
    if (ID == 0xD3D3) ID = 0x9486; // write-only shield
    tft.begin(ID);
    tft.setRotation(1);
    tft.fillScreen(BLACK);
 width = tft.width(); // 480
 height = tft.height(); // 320
    hh = conv2d(__TIME__);
    mm = conv2d(__TIME__ + 3);
    ss = conv2d(__TIME__ + 6);

}
void Ramka(void){
    for (y = 0, w = 18, h = 3; y < height; y += height - h) {
    for (x = 14; x < width - w; x += 2 * w) {
      tft.fillRect(x, y, w, h, WHITE);
      }
      }

  for (x = 0, w = 7, h = 18; x < width; x += width - w ) {
    for (y = 21; y < height - h; y += 2 * h) {
      tft.fillRect(x, y, w, h, WHITE);
      }
      }
}

void Pokraym(){
  
    tft.fillRect(0, 0, 7, 3, WHITE); //W L
  tft.fillRect(width - 7, 0, 7, 3, WHITE); // W R
  tft.fillRect(0, height - 3, 7, 3, WHITE); // N L
  tft.fillRect(width - 7, height - 3, 7, 3, WHITE); // N R
}
void loop(void)
{
 //width = tft.width(); // 480
// height = tft.height(); // 320
Pokraym();
Ramka();
     tft.fillRect(7, 3, 233 , 17 * 18, GREY);
    for (x = 7, y = 0, w = 1, h = height; x < width; x += 18) {
        tft.fillRect(x, y, w, h, WHITE);
    }
    for (x = 0, y = 3, w = width, h = 1; y < height; y += 18) {
        tft.fillRect(x, y, w, h, WHITE);
    }

     int _H = height / 2 ; // 160
    int _W = width / 2; // 240
    tft.fillRect(26, 22, 17, _H - 22, TURQUOISE);
    tft.fillRect(26, _H, 17, _H - 22, PINK);
    tft.fillRect(44, 22, 17, 45, AZURE);
    tft.fillRect(44, 253, 17, 45, ORANGE);
    tft.fillRect(423, 22, 17, 45, AZURE);
    tft.fillRect(423, 253, 17, 45, ORANGE);
   tft.fillRect(440, 22, 17, _H - 22, OLIVE);
   tft.fillRect(440, _H, 17, _H - 22, PURPLE);


    for (dx = radius; dx > -radius; dx--) {
        w = sqrt(radius * radius - dx * dx );
        y = 160 - dx;
        dy = (y - 3) / 18;
        mask = 7;
        colors[0] = (dy == 3) ? DARKGREY : BLACK;
        switch (dy) {
            case 0:
            case 1: idx = 1; len = 0; break;
            case 2: idx = 0; len = 0; break;
            case 3: idx = 0; len = 13; mask = 1; break;
            case 4:
            case 5: idx = 2; len = 38; break;
            case 6:
            case 7:
            case 8: idx = 0; len = 0; break;
            case 9: for (idx = 2; idx < 8; idx++) {
                    //dy = 0xFF >> (7 - idx);
                    dy = (idx - 2) * 51;
                    colors[idx] = tft.color565(dy, dy, dy);
                }
                idx = 2; len = 38; break;
            case 10: idx = 1; len = 0; break;
            case 11:
            case 12: colors[2] = YELLOW; idx = 2; len = 0; break;
        }
        if (len == 0)
            tft.fillRect(_W - w, y, w * 2, 1, colors[idx]);

        else {
            if (mask == 1) idx = 1 + (w) / len;
            dy = w % len;
            for (x = _W - w; x < _W + w; idx++) {
                tft.fillRect(x, y, dy, 1, colors[idx & mask]);
                x += dy;
                if (x + len > _W + w) dy = w % len;
                else dy = len;
            }
        }

    }
    for (x = _W - 140, y = 129, dx = 5, dy = 0; dx > 0; x += 2 * dx) {
        tft.fillRect(x, y, dx, 36, WHITE);
        dy += dx * 2;
        if (dy >= 36) {
            dy = 0;
            dx--;
        }
    }
    tft.fillRect(_W - 8, 5 * 18 + 3, 17, 3 * 18, BLACK);
    for (x = 3 * 18 + 7, y = 6 * 18 + 3, w = 1, h = 18; x < _W + 108; x += 18) {
        tft.fillRect(x, y, w, h, WHITE);
    }
    tft.fillRect(_W - 108, _H, 108 * 2, 1, WHITE);
    tft.fillRect(_W, 5 * 18 + 3, 1, 3 * 18, WHITE);
    tft.fillRect(186, 2 * 18 + 3, 6 * 18, 18, WHITE);
    //    tft.fillRect(108, 10 * 18 + 3, 6 * 18, 18, BLACK);
    tft.fillRect(_W - 8, 11 * 18 + 3, 17, radius - 18*9/2, RED);
    tft.setCursor(_W - 16, 24 + ADJ_BASELINE);
    tft.setTextColor(BLACK);
    tft.setTextSize(1);
    tft.print("480x320");
    tft.setCursor(216, 43 + ADJ_BASELINE);
    tft.setTextColor(BLACK);
    tft.setTextSize(1);
    tft.print("ID=0x");
    tft.print(tft.readID(), HEX);
    tft.setTextColor(WHITE, BLACK);
    //    tft.setFont(NULL);
    //    tft.setTextSize(2);
    while (1) {
        if (++ss > 59) {
            ss = 0;
            mm++;
            if (mm > 59) {
                mm = 0;
                hh++;
                if (hh > 23) hh = 0;
            }
        }
        char buf[20];
        sprintf(buf, "%02d:%02d:%02d", hh, mm, ss);
        tft.fillRect(186, 10 * 18 + 3, 6 * 18, 18, BLACK);
        tft.setCursor(216, 187 + ADJ_BASELINE);
        tft.print(buf);
        delay(1000);
    }
}