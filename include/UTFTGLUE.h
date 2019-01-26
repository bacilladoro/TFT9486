/*
 * utftglue.h
 *
 * Created: 02/03/2013 14:25:06
 *  Author: David Prentice
 */ 


#ifndef UTFTGLUE_H_
#define UTFTGLUE_H_

#define LEFT 0
#define RIGHT 9999
#define CENTER 9998

#define PORTRAIT 0
#define LANDSCAPE 1

#include <mcutft.h>

#include <Fruit_GFX.h>
//#if !defined(AVR) && !defined(ESP32)
#if defined(__arm)
#include <avr/dtostrf.h>
#endif

#undef _GFXFONT_H_     //comment this line to enable FreeFonts
#if defined(_GFXFONT_H_)
#include <./FreeDefaultFonts.h>
#define SmallFont &FreeSmallFont
#define BigFont   &FreeBigFont
#define SevenSegNumFont &FreeSevenSegNumFont
#else
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];
#endif

struct _current_font
{
	uint8_t* font;
	uint8_t x_size;
	uint8_t y_size;
	uint8_t offset;
	uint8_t numchars;
};

class UTFTGLUE : public mcutft
{
	public:
//	UTFTGLUE() : mcutft() {}
	UTFTGLUE(byte model, int RS, int WR,int CS, int RST, int RD = A0) : mcutft(CS, RS, WR, RD, RST) {}
	void InitLCD(byte orientation=LANDSCAPE) {
	     mcutft::reset();
		 uint16_t ID = mcutft::readID();
		 if (ID == 0) ID = 0x9341;        //DealExtreme with EXTC=0
//		 if (ID == 0x0089 || ID == 0x8989) ID = 0x1289;
//		 if (ID == 0x00D3 || ID == 0xD3D3) ID = 0x9481;   //write-only controller
//		 if (ID == 0x00D3 || ID == 0xD3D3) ID = 0x9486;   //write-only controller
		 if (ID == 0x00D3 || ID == 0xD3D3) ID = 0x9090;   //write-only controller HX8357-D
//         if (ID == 0x9327 && orientation == LANDSCAPE) orientation = 3;
		 mcutft::begin(ID);
		 mcutft::setRotation(_orient = orientation);
		 _radius = 4;
		 _fontsize = 1;
    }
	void clrScr() { mcutft::fillScreen(0x0000);}
	void drawPixel(int x, int y) { mcutft::drawPixel(x, y, _fcolor);}
	void drawLine(int x1, int y1, int x2, int y2) { mcutft::drawLine(x1, y1, x2, y2, _fcolor);}
	void fillScr(byte r, byte g, byte b) { mcutft::fillScreen(setrgb(r, g, b));}
	void drawRect(int x1, int y1, int x2, int y2) {
		 int w = x2 - x1 + 1, h = y2 - y1 + 1;
	     if (w < 0) { x1 = x2; w = -w; }
         if (h < 0) { y1 = y2; h = -h; }
         mcutft::drawRect(x1, y1, w, h, _fcolor);
    }
	void drawRoundRect(int x1, int y1, int x2, int y2) {
		 int w = x2 - x1 + 1, h = y2 - y1 + 1;
	     if (w < 0) { x1 = x2; w = -w; }
         if (h < 0) { y1 = y2; h = -h; }
         mcutft::drawRoundRect(x1, y1, w, h, _radius, _fcolor);
    }
	void fillRect(int x1, int y1, int x2, int y2) {
		 int w = x2 - x1 + 1, h = y2 - y1 + 1;
    	 if (w < 0) { x1 = x2; w = -w; }
		 if (h < 0) { y1 = y2; h = -h; }
         mcutft::fillRect(x1, y1, w, h, _fcolor);
    }
	void fillRoundRect(int x1, int y1, int x2, int y2) {
		 int w = x2 - x1 + 1, h = y2 - y1 + 1;
	     if (w < 0) { x1 = x2; w = -w; }
         if (h < 0) { y1 = y2; h = -h; }
         mcutft::fillRoundRect(x1, y1, w, h, _radius, _fcolor);
    }
	void drawCircle(int x, int y, int radius) { mcutft::drawCircle(x, y, radius, _fcolor);}
	void fillCircle(int x, int y, int radius) { mcutft::fillCircle(x, y, radius, _fcolor);}
	void setColor(byte r, byte g, byte b)  { mcutft::setTextColor(_fcolor = setrgb(r, g, b), _bcolor);}
	void setColor(uint16_t c)  { mcutft::setTextColor(_fcolor = c, _bcolor);}
	void setBackColor(byte r, byte g, byte b)  { mcutft::setTextColor(_fcolor, _bcolor = setrgb(r, g, b));}
	void setBackColor(uint32_t c)  { if (c == 0xFFFFFFFF) _bcolor = _fcolor; mcutft::setTextColor(_fcolor, _bcolor = c);}
	void print(const char *st, int x, int y, int deg=0)  {
		 settextcursor((char*)st, x, y); mcutft::print(st);}
	void print(char *st, int x, int y, int deg=0)  {
		 settextcursor(st, x, y); mcutft::print(st);}
	void print(String st, int x, int y, int deg=0) {
		 settextcursor((char*)(st.c_str()), x, y); mcutft::print(st);}
	void printNumI(long num, int x, int y, int length=0, char filler=' ') {
//		 char buf[17]; ltoa(num, buf, 10); 
//		 settextcursor(buf, x, y); mcutft::print(buf);
//		 settextcursor((char*)"", x, y); mcutft::print(num);
         char buf[17], len = 0, *p = buf + 17, sign = num < 0;
		 if (sign) num = -num;
		 *--p = '\0';
         if (num == 0) *--p = '0';
		 else while (num > 0 && len < 10) {  //int32_t -2147483648 i.e. 10 digits + sign
             *--p = '0' + num % 10;
             num /= 10;
             len++;
         }
		 if (sign) *--p = '-';
		 while (len++ < length) *--p = filler;
		 settextcursor(p, x, y);     //calculates where to start cursor for CENTER, RIGHT etc
         mcutft::print(p);    //actually print the data
    }
	void printNumF(double num, byte dec, int x, int y, char divider='.', int length=0, char filler=' ') {
//		 settextcursor((char*)"", x, y); mcutft::print(num, dec);
		 char buf[20]; dtostrf(num, length, dec, buf); settextcursor(buf, x, y); mcutft::print(buf);
		 }
#if !defined(_GFXFONT_H_)
	void setFont(uint8_t* font) { mcutft::setTextSize(_fontsize = 1);}
#endif
	void drawBitmap(int x, int y, int sx, int sy, const uint16_t *data, int scale=1) {
		 uint16_t color;
		 mcutft::setAddrWindow(x, y, x + sx*scale - 1, y + sy*scale - 1);
		 if (scale == 1) mcutft::pushColors((const uint8_t*)data, sx * sy, 1);
		 else {
		     for (int row = 0; row < sy; row++) {
			     for (int col = 0; col < sx; col++) {
                     color = pgm_read_word(data + (row*sx + col)*1);
					 mcutft::fillRect(x+col*scale, y+row*scale, scale, scale, color);
			     }
		     }
		 }
		 }
//	void drawBitmap(int x, int y, int sx, int sy, bitmapdatatype data, int deg, int rox, int roy);
//	void lcdOff();
//	void lcdOn();
//	void setContrast(char c);
	int  getDisplayXSize() { return mcutft::width(); }
	int	 getDisplayYSize() { return mcutft::height(); }
//	void LCD_Write_DATA(char VH,char VL);
	//		void dispBitmap(File inFile);
	protected:
	uint16_t _fcolor;
	uint16_t _bcolor;
	uint8_t _fontsize;
	uint8_t _radius;
	uint8_t _orient;
    void settextcursor(char *st, int x, int y) {
		int16_t pos, x1, y1;
		uint16_t len, w, h;
#if defined(_GFXFONT_H_)
		if (mcutft::gfxFont != NULL) y += pgm_read_byte(&gfxFont->yAdvance);
		if (x == CENTER || x == RIGHT) {
		    if (mcutft::gfxFont != NULL) {
			    getTextBounds(st, 0, 0, &x1, &y1, &w, &h);
				len = x1 - 0 + w;    // misses the final right padding.
		    } else 
#else
		if (x == CENTER || x == RIGHT) {
#endif
			{
			    len = strlen(st) * 6 * _fontsize;
			}
			pos = (mcutft::width() - len);
			if (x == CENTER) x = pos/2;
			else x = pos - 1;
		}			
        mcutft::setCursor(x, y);
	}
    uint16_t setrgb(byte r, byte g, byte b)  { return ((r&0xF8) << 8) | ((g&0xFC) << 3) | (b>>3);}
};

#endif /* UTFTGLUE_H_ */