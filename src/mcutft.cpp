#define SUPPORT_9806 

#include "mcutft.h"
#include "mcutft_shield.h"

#define MIPI_DCS_REV1   (1<<0)
#define AUTO_READINC    (1<<1)
#define READ_BGR        (1<<2)
#define READ_LOWHIGH    (1<<3)
#define READ_24BITS     (1<<4)
#define XSA_XEA_16BIT   (1<<5)
#define READ_NODUMMY    (1<<6)
#define INVERT_GS       (1<<8)
#define INVERT_SS       (1<<9)
#define MV_AXIS         (1<<10)
#define INVERT_RGB      (1<<11)
#define REV_SCREEN      (1<<12)
#define FLIP_VERT       (1<<13)
#define FLIP_HORIZ      (1<<14)

#if (defined(USES_16BIT_BUS))   //only comes from SPECIALs
#define USING_16BIT_BUS 1
#else
#define USING_16BIT_BUS 0
#endif

mcutft::mcutft(int CS, int RS, int WR, int RD, int _RST):Fruit_GFX(240, 320)
{
    // we can not access GPIO pins until AHB has been enabled.
}

static uint8_t done_reset, is8347, is555;
static uint16_t color565_to_555(uint16_t color) {
    return (color & 0xFFC0) | ((color & 0x1F) << 1) | ((color & 0x01));  //lose Green LSB, extend Blue LSB
}
static uint16_t color555_to_565(uint16_t color) {
    return (color & 0xFFC0) | ((color & 0x0400) >> 5) | ((color & 0x3F) >> 1); //extend Green LSB
}

void mcutft::reset(void)
{
    done_reset = 1;
    setWriteDir();
    CTL_INIT();
    CS_IDLE;
    RD_IDLE;
    WR_IDLE;
    RESET_IDLE;
    delay(50);
    RESET_ACTIVE;
    delay(100);
    RESET_IDLE;
    delay(100);
	WriteCmdData(0xB0, 0x0000);   //R61520 needs this to read ID
}

static void writecmddata(uint16_t cmd, uint16_t dat)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}

void mcutft::WriteCmdData(uint16_t cmd, uint16_t dat) { writecmddata(cmd, dat); }

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    while (N-- > 0) {
        uint8_t u8 = *block++;
        write8(u8);
        if (N && is8347) {
            cmd++;
            WriteCmd(cmd);
        }
    }
    CS_IDLE;
}

static inline void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
    uint8_t d[4];
    d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;
    WriteCmdParamN(cmd, 4, d);
}

//#define WriteCmdParam4(cmd, d1, d2, d3, d4) {uint8_t d[4];d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;WriteCmdParamN(cmd, 4, d);}
void mcutft::pushCommand(uint16_t cmd, uint8_t * block, int8_t N) { WriteCmdParamN(cmd, N, block); }

static uint16_t read16bits(void)
{
    uint16_t ret;
    uint8_t lo;
#if USING_16BIT_BUS
    READ_16(ret);               //single strobe to read whole bus
    if (ret > 255)              //ID might say 0x00D3
        return ret;
#else
    READ_8(ret);
#endif
    //all MIPI_DCS_REV1 style params are 8-bit
    READ_8(lo);
    return (ret << 8) | lo;
}

uint16_t mcutft::readReg(uint16_t reg, int8_t index)
{
    uint16_t ret;
    uint8_t lo;
    if (!done_reset)
        reset();
    CS_ACTIVE;
    WriteCmd(reg);
    setReadDir();
    delay(1);    //1us should be adequate
    //    READ_16(ret);
    do { ret = read16bits(); }while (--index >= 0);  //need to test with SSD1963
    RD_IDLE;
    CS_IDLE;
    setWriteDir();
    return ret;
}

uint32_t mcutft::readReg32(uint16_t reg)
{
    uint16_t h = readReg(reg, 0);
    uint16_t l = readReg(reg, 1);
    return ((uint32_t) h << 16) | (l);
}

uint32_t mcutft::readReg40(uint16_t reg)
{
    uint16_t h = readReg(reg, 0);
    uint16_t m = readReg(reg, 1);
    uint16_t l = readReg(reg, 2);
    return ((uint32_t) h << 24) | (m << 8) | (l >> 8);
}

uint16_t mcutft::readID(void)
{
    uint16_t ret, ret2;
    uint8_t msb;
    ret = readReg(0);           //forces a reset() if called before begin()


    uint32_t ret32 = readReg32(0x04);
    msb = ret32 >> 16;
    ret = ret32;	


    ret = readReg32(0xD3);      //for ILI9488, 9486, 9340, 9341
    msb = ret >> 8;
    if (msb == 0x93 || msb == 0x94 || msb == 0x98 || msb == 0x77 || msb == 0x16)
        return ret;             //0x9488, 9486, 9340, 9341, 7796
    if (ret == 0x00D3 || ret == 0xD3D3)
        return ret;             //16-bit write-only bus

	return readReg(0);          //0154, 7783, 9320, 9325, 9335, B505, B509
}

 // independent cursor and window registers.   S6D0154, ST7781 increments.  ILI92320/5 do not.  
int16_t mcutft::readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
    uint16_t ret, dummy, _MR = _MW;
    int16_t n = w * h, row = 0, col = 0;
    uint8_t r, g, b, tmp;
    if (!is8347 && _lcd_capable & MIPI_DCS_REV1) // HX8347 uses same register
        _MR = 0x2E;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    while (n > 0) {
        if (!(_lcd_capable & MIPI_DCS_REV1)) {
            WriteCmdData(_MC, x + col);
            WriteCmdData(_MP, y + row);
        }
        CS_ACTIVE;
        WriteCmd(_MR);
        setReadDir();
        if (_lcd_capable & READ_NODUMMY) {
            ;
        } else if ((_lcd_capable & MIPI_DCS_REV1) || _lcd_ID == 0x1289) {
            READ_8(r);
        } else {
            READ_16(dummy);
        }
		if (_lcd_ID == 0x1511) READ_8(r);   //extra dummy for R61511
        while (n) {
            if (_lcd_capable & READ_24BITS) {
                READ_8(r);
                READ_8(g);
                READ_8(b);
                if (_lcd_capable & READ_BGR)
                    ret = color565(b, g, r);
                else
                    ret = color565(r, g, b);
            } else {
                READ_16(ret);
                if (_lcd_capable & READ_LOWHIGH)
                    ret = (ret >> 8) | (ret << 8);
                if (_lcd_capable & READ_BGR)
                    ret = (ret & 0x07E0) | (ret >> 11) | (ret << 11);
            }
#if defined(SUPPORT_9488_555)
    if (is555) ret = color555_to_565(ret);
#endif
            *block++ = ret;
            n--;
            if (!(_lcd_capable & AUTO_READINC))
                break;
        }
        if (++col >= w) {
            col = 0;
            if (++row >= h)
                row = 0;
        }
        RD_IDLE;
        CS_IDLE;
        setWriteDir();
    }
    if (!(_lcd_capable & MIPI_DCS_REV1))
        setAddrWindow(0, 0, width() - 1, height() - 1);
    return 0;
}

void mcutft::setRotation(uint8_t r)
{
    uint16_t GS, SS_v, ORG, REV = _lcd_rev;
    uint8_t val, d[3];
    rotation = r & 3;           // just perform the operation ourselves on the protected variables
    _width = (rotation & 1) ? HEIGHT : WIDTH;
    _height = (rotation & 1) ? WIDTH : HEIGHT;
    switch (rotation) {
    case 0:                    //PORTRAIT:
        val = 0x48;             //MY=0, MX=1, MV=0, ML=0, BGR=1
        break;
    case 1:                    //LANDSCAPE: 90 degrees
        val = 0x28;             //MY=0, MX=0, MV=1, ML=0, BGR=1
        break;
    case 2:                    //PORTRAIT_REV: 180 degrees
        val = 0x98;             //MY=1, MX=0, MV=0, ML=1, BGR=1
        break;
    case 3:                    //LANDSCAPE_REV: 270 degrees
        val = 0xF8;             //MY=1, MX=1, MV=1, ML=1, BGR=1
        break;
    }
    if (_lcd_capable & INVERT_GS)
        val ^= 0x80;
    if (_lcd_capable & INVERT_SS)
        val ^= 0x40;
    if (_lcd_capable & INVERT_RGB)
        val ^= 0x08;
    if (_lcd_capable & MIPI_DCS_REV1) {
        if (_lcd_ID == 0x6814) {  //.kbv my weird 0x9486 might be 68140
            GS = (val & 0x80) ? (1 << 6) : 0;   //MY
            SS_v = (val & 0x40) ? (1 << 5) : 0;   //MX
            val &= 0x28;        //keep MV, BGR, MY=0, MX=0, ML=0
            d[0] = 0;
            d[1] = GS | SS_v | 0x02;      //MY, MX
            d[2] = 0x3B;
            WriteCmdParamN(0xB6, 3, d);
            goto common_MC;
        } else if (_lcd_ID == 0x1963 || _lcd_ID == 0x9481 || _lcd_ID == 0x1511) {
            if (val & 0x80)
                val |= 0x01;    //GS
            if ((val & 0x40))
                val |= 0x02;    //SS
            if (_lcd_ID == 0x1963) val &= ~0xC0;
            if (_lcd_ID == 0x9481) val &= ~0xD0;
            if (_lcd_ID == 0x1511) {
                val &= ~0x10;   //remove ML
                val |= 0xC0;    //force penguin 180 rotation
            }
//            val &= (_lcd_ID == 0x1963) ? ~0xC0 : ~0xD0; //MY=0, MX=0 with ML=0 for ILI9481
            goto common_MC;
        } else if (is8347) {
            _MC = 0x02, _MP = 0x06, _MW = 0x22, _SC = 0x02, _EC = 0x04, _SP = 0x06, _EP = 0x08;
            if (_lcd_ID == 0x0065) {             //HX8352-B
                if (!(val & 0x10)) val ^= 0x81;  //(!ML) flip MY, GS
                if (r & 1) _MC = 0x82, _MP = 0x80;
                else _MC = 0x80, _MP = 0x82;
            }
			if (_lcd_ID == 0x5252) {
			    val |= 0x02;   //VERT_SCROLLON
				if (val & 0x10) val |= 0x04;   //if (ML) SS=1 kludge mirror in XXX_REV modes
            }
			goto common_BGR;
        }
      common_MC:
        _MC = 0x2A, _MP = 0x2B, _MW = 0x2C, _SC = 0x2A, _EC = 0x2A, _SP = 0x2B, _EP = 0x2B;
      common_BGR:
        WriteCmdParamN(is8347 ? 0x16 : 0x36, 1, &val);
        _lcd_madctl = val;
//	    if (_lcd_ID	== 0x1963) WriteCmdParamN(0x13, 0, NULL);   //NORMAL mode
    }
    // cope with 9320 variants
    else {
        switch (_lcd_ID) {
#if defined(SUPPORT_9225)
        case 0x9225:
        case 0x9226:
            _SC = 0x37, _EC = 0x36, _SP = 0x39, _EP = 0x38;
            _MC = 0x20, _MP = 0x21, _MW = 0x22;
            GS = (val & 0x80) ? (1 << 9) : 0;
            SS_v = (val & 0x40) ? (1 << 8) : 0;
            WriteCmdData(0x01, GS | SS_v | 0x001C);       // set Driver Output Control
            goto common_ORG;
#endif
#if defined(SUPPORT_0139) || defined(SUPPORT_0154)
#ifdef SUPPORT_0139
        case 0x0139:
            _SC = 0x46, _EC = 0x46, _SP = 0x48, _EP = 0x47;
            goto common_S6D;
#endif
#ifdef SUPPORT_0154
        case 0x0154:
            _SC = 0x37, _EC = 0x36, _SP = 0x39, _EP = 0x38;
            goto common_S6D;
#endif
          common_S6D:
            _MC = 0x20, _MP = 0x21, _MW = 0x22;
            GS = (val & 0x80) ? (1 << 9) : 0;
            SS_v = (val & 0x40) ? (1 << 8) : 0;
            // S6D0139 requires NL = 0x27,  S6D0154 NL = 0x28
            WriteCmdData(0x01, GS | SS_v | ((_lcd_ID == 0x0139) ? 0x27 : 0x28));
            goto common_ORG;
#endif
        case 0x5420:
        case 0x7793:
        case 0x9326:
		case 0xB509:
            _MC = 0x200, _MP = 0x201, _MW = 0x202, _SC = 0x210, _EC = 0x211, _SP = 0x212, _EP = 0x213;
            GS = (val & 0x80) ? (1 << 15) : 0;
			uint16_t NL;
			NL = ((432 / 8) - 1) << 9;
            if (_lcd_ID == 0x9326 || _lcd_ID == 0x5420) NL >>= 1;
            WriteCmdData(0x400, GS | NL);
            goto common_SS;
        default:
            _MC = 0x20, _MP = 0x21, _MW = 0x22, _SC = 0x50, _EC = 0x51, _SP = 0x52, _EP = 0x53;
            GS = (val & 0x80) ? (1 << 15) : 0;
            WriteCmdData(0x60, GS | 0x2700);    // Gate Scan Line (0xA700)
          common_SS:
            SS_v = (val & 0x40) ? (1 << 8) : 0;
            WriteCmdData(0x01, SS_v);     // set Driver Output Control
          common_ORG:
            ORG = (val & 0x20) ? (1 << 3) : 0;
#ifdef SUPPORT_8230
            if (_lcd_ID == 0x8230) {    // UC8230 has strange BGR and READ_BGR behaviour
                if (rotation == 1 || rotation == 2) {
                    val ^= 0x08;        // change BGR bit for LANDSCAPE and PORTRAIT_REV
                }
            }               
#endif
            if (val & 0x08)
                ORG |= 0x1000;  //BGR
            _lcd_madctl = ORG | 0x0030;
            WriteCmdData(0x03, _lcd_madctl);    // set GRAM write direction and BGR=1.
            break;
#ifdef SUPPORT_1289
        case 0x1289:
            _MC = 0x4E, _MP = 0x4F, _MW = 0x22, _SC = 0x44, _EC = 0x44, _SP = 0x45, _EP = 0x46;
            if (rotation & 1)
                val ^= 0xD0;    // exchange Landscape modes
            GS = (val & 0x80) ? (1 << 14) | (1 << 12) : 0;      //called TB (top-bottom)
            SS_v = (val & 0x40) ? (1 << 9) : 0;   //called RL (right-left)
            ORG = (val & 0x20) ? (1 << 3) : 0;  //called AM
            _lcd_drivOut = GS | SS_v | (REV << 13) | 0x013F;      //REV=0, BGR=0, MUX=319
            if (val & 0x08)
                _lcd_drivOut |= 0x0800; //BGR
            WriteCmdData(0x01, _lcd_drivOut);   // set Driver Output Control
            WriteCmdData(0x11, ORG | 0x6070);   // set GRAM write direction.
            break;
#endif
		}
    }
    if ((rotation & 1) && ((_lcd_capable & MV_AXIS) == 0)) {
        uint16_t x;
        x = _MC, _MC = _MP, _MP = x;
        x = _SC, _SC = _SP, _SP = x;    //.kbv check 0139
        x = _EC, _EC = _EP, _EP = x;    //.kbv check 0139
    }
    setAddrWindow(0, 0, width() - 1, height() - 1);
    vertScroll(0, HEIGHT, 0);   //reset scrolling after a rotation
}

void mcutft::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // MCUFRIEND just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
#if defined(SUPPORT_9488_555)
    if (is555) color = color565_to_555(color);
#endif
    setAddrWindow(x, y, x, y);
//    CS_ACTIVE; WriteCmd(_MW); write16(color); CS_IDLE; //-0.01s +98B
    WriteCmdData(_MW, color);
}

void mcutft::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
#if defined(OFFSET_9327)
	if (_lcd_ID == 0x9327) {
	    if (rotation == 2) y += OFFSET_9327, y1 += OFFSET_9327;
	    if (rotation == 3) x += OFFSET_9327, x1 += OFFSET_9327;
    }
#endif
#if 1
    if (_lcd_ID == 0x1526 && (rotation & 1)) {
		int16_t dx = x1 - x, dy = y1 - y;
		if (dy == 0) { y1++; }
		else if (dx == 0) { x1 += dy; y1 -= dy; }
    }
#endif
    if (_lcd_capable & MIPI_DCS_REV1) {
        WriteCmdParam4(_SC, x >> 8, x, x1 >> 8, x1);   //Start column instead of _MC
        WriteCmdParam4(_SP, y >> 8, y, y1 >> 8, y1);   //
        if (is8347 && _lcd_ID == 0x0065) {             //HX8352-B has separate _MC, _SC
            uint8_t d[2];
            d[0] = x >> 8; d[1] = x;
            WriteCmdParamN(_MC, 2, d);                 //allows !MV_AXIS to work
            d[0] = y >> 8; d[1] = y;
            WriteCmdParamN(_MP, 2, d);
        }
    } else {
        WriteCmdData(_MC, x);
        WriteCmdData(_MP, y);
        if (!(x == x1 && y == y1)) {  //only need MC,MP for drawPixel
            if (_lcd_capable & XSA_XEA_16BIT) {
                if (rotation & 1)
                    y1 = y = (y1 << 8) | y;
                else
                    x1 = x = (x1 << 8) | x;
            }
            WriteCmdData(_SC, x);
            WriteCmdData(_SP, y);
            WriteCmdData(_EC, x1);
            WriteCmdData(_EP, y1);
        }
    }
}

void mcutft::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    int16_t end;
#if defined(SUPPORT_9488_555)
    if (is555) color = color565_to_555(color);
#endif
    if (w < 0) {
        w = -w;
        x -= w;
    }                           //+ve w
    end = x + w;
    if (x < 0)
        x = 0;
    if (end > width())
        end = width();
    w = end - x;
    if (h < 0) {
        h = -h;
        y -= h;
    }                           //+ve h
    end = y + h;
    if (y < 0)
        y = 0;
    if (end > height())
        end = height();
    h = end - y;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    CS_ACTIVE;
    WriteCmd(_MW);
    if (h > w) {
        end = h;
        h = w;
        w = end;
    }
    uint8_t hi = color >> 8, lo = color & 0xFF;
    while (h-- > 0) {
        end = w;
#if USING_16BIT_BUS
#if defined(__MK66FX1M0__)      //180MHz M4
#define STROBE_16BIT {WR_ACTIVE4;WR_ACTIVE;WR_IDLE4;WR_IDLE;}   //56ns
#elif defined(__SAM3X8E__)      //84MHz M3
#define STROBE_16BIT {WR_ACTIVE4;WR_ACTIVE2;WR_IDLE4;WR_IDLE2;} //286ns ?ILI9486
//#define STROBE_16BIT {WR_ACTIVE4;WR_ACTIVE;WR_IDLE4;WR_IDLE;} //238ns SSD1289
//#define STROBE_16BIT {WR_ACTIVE2;WR_ACTIVE;WR_IDLE2;}      //119ns RM68140
#else                           //16MHz AVR
#define STROBE_16BIT {WR_ACTIVE;WR_ACTIVE;WR_IDLE; }            //375ns ?ILI9486
#endif
        write_16(color);        //we could just do the strobe
        lo = end & 7;
        hi = end >> 3;
        if (hi)
            do {
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
            } while (--hi > 0);
        while (lo-- > 0) {
            STROBE_16BIT;
        }
#else
        do {
            write8(hi);
            write8(lo);
        } while (--end != 0);
#endif
    }
    CS_IDLE;
    if (!(_lcd_capable & MIPI_DCS_REV1) || ((_lcd_ID == 0x1526) && (rotation & 1)))
        setAddrWindow(0, 0, width() - 1, height() - 1);
}

static void pushColors_any(uint16_t cmd, uint8_t * block, int16_t n, bool first, uint8_t flags)
{
    uint16_t color;
    uint8_t h, l;
	bool isconst = flags & 1;
	bool isbigend = (flags & 2) != 0;
    CS_ACTIVE;
    if (first) {
        WriteCmd(cmd);
    }
    while (n-- > 0) {
        if (isconst) {
            h = pgm_read_byte(block++);
            l = pgm_read_byte(block++);
        } else {
		    h = (*block++);
            l = (*block++);
		}
        color = (isbigend) ? (h << 8 | l) :  (l << 8 | h);
#if defined(SUPPORT_9488_555)
    if (is555) color = color565_to_555(color);
#endif
        write16(color);
    }
    CS_IDLE;
}

void mcutft::pushColors(uint16_t * block, int16_t n, bool first)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, 0);
}
void mcutft::pushColors(uint8_t * block, int16_t n, bool first)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, 2);   //regular bigend
}
void mcutft::pushColors(const uint8_t * block, int16_t n, bool first, bool bigend)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, bigend ? 3 : 1);
}

void mcutft::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{

    int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
    int16_t vsp;
    int16_t sea = top;
	if (_lcd_ID == 0x9327) bfa += 32;
    if (offset <= -scrollines || offset >= scrollines) offset = 0; //valid scroll
	vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    sea = top + scrollines - 1;
    if (_lcd_capable & MIPI_DCS_REV1) {
        uint8_t d[6];           // for multi-byte parameters

        d[0] = top >> 8;        //TFA
        d[1] = top;
        d[2] = scrollines >> 8; //VSA
        d[3] = scrollines;
        d[4] = bfa >> 8;        //BFA
        d[5] = bfa;
        WriteCmdParamN(is8347 ? 0x0E : 0x33, 6, d);
//        if (offset == 0 && rotation > 1) vsp = top + scrollines;   //make non-valid
		d[0] = vsp >> 8;        //VSP
        d[1] = vsp;
        WriteCmdParamN(is8347 ? 0x14 : 0x37, 2, d);
		if (is8347) { 
		    d[0] = (offset != 0) ? (_lcd_ID == 0x8347 ? 0x02 : 0x08) : 0;
			WriteCmdParamN(_lcd_ID == 0x8347 ? 0x18 : 0x01, 1, d);  //HX8347-D
		} else if (offset == 0 && (_lcd_capable & MIPI_DCS_REV1)) {
			WriteCmdParamN(0x13, 0, NULL);    //NORMAL i.e. disable scroll
		}
		return;
    }

}

void mcutft::invertDisplay(boolean i)
{
    uint8_t val;
    _lcd_rev = ((_lcd_capable & REV_SCREEN) != 0) ^ i;
    if (_lcd_capable & MIPI_DCS_REV1) {
        
            WriteCmdParamN(_lcd_rev ? 0x21 : 0x20, 0, NULL);
        return;
    }

}

#define TFTLCD_DELAY 0xFFFF
#define TFTLCD_DELAY8 0x7F
static void init_table(const void *table, int16_t size)
{
#ifdef SUPPORT_8357D_GAMMA
    uint8_t *p = (uint8_t *) table, dat[36];            //HX8357_99 has GAMMA[34] 
#else
    uint8_t *p = (uint8_t *) table, dat[24];            //R61526 has GAMMA[22] 
#endif
    while (size > 0) {
        uint8_t cmd = pgm_read_byte(p++);
        uint8_t len = pgm_read_byte(p++);
        if (cmd == TFTLCD_DELAY8) {
            delay(len);
            len = 0;
        } else {
            for (uint8_t i = 0; i < len; i++)
                dat[i] = pgm_read_byte(p++);
            WriteCmdParamN(cmd, len, dat);
        }
        size -= len + 2;
    }
}

static void init_table16(const void *table, int16_t size)
{
    uint16_t *p = (uint16_t *) table;
    while (size > 0) {
        uint16_t cmd = pgm_read_word(p++);
        uint16_t d = pgm_read_word(p++);
        if (cmd == TFTLCD_DELAY)
            delay(d);
        else {
			writecmddata(cmd, d);                      //static function
        }
        size -= 2 * sizeof(int16_t);
    }
}

void mcutft::begin(uint16_t ID)
{
    int16_t *p16;               //so we can "write" to a const protected variable.
    const uint8_t *table8_ads = NULL;
    int16_t table_size;
    reset();
    _lcd_xor = 0;
    if (ID = 9486) {

        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS; //Red 3.5", Blue 3.5"
//        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | REV_SCREEN; //old Red 3.5"
        static const uint8_t ILI9486_regValues[] PROGMEM = {
/*
            0xF2, 9, 0x1C, 0xA3, 0x32, 0x02, 0xB2, 0x12, 0xFF, 0x12, 0x00,        //f.k
            0xF1, 2, 0x36, 0xA4,        //
            0xF8, 2, 0x21, 0x04,        //
            0xF9, 2, 0x00, 0x08,        //
*/
            0xC0, 2, 0x0d, 0x0d,        //Power Control 1 [0E 0E]
            0xC1, 2, 0x43, 0x00,        //Power Control 2 [43 00]
            0xC2, 1, 0x00,      //Power Control 3 [33]
            0xC5, 4, 0x00, 0x48, 0x00, 0x48,    //VCOM  Control 1 [00 40 00 40]
            0xB4, 1, 0x00,      //Inversion Control [00]
            0xB6, 3, 0x02, 0x02, 0x3B,  // Display Function Control [02 02 3B]
#define GAMMA9486 4
#if GAMMA9486 == 0
            // default GAMMA terrible
#elif GAMMA9486 == 1
            // GAMMA f.k.	bad		
            0xE0, 15, 0x0f, 0x31, 0x2b, 0x0c, 0x0e, 0x08, 0x4e, 0xf1, 0x37, 0x07, 0x10, 0x03, 0x0e, 0x09, 0x00,
            0xE1, 15, 0x00, 0x0e, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0f, 0x0c, 0x31, 0x36, 0x0f,
#elif GAMMA9486 == 2
            // 1.2 CPT 3.5 Inch Initial Code not bad
			0xE0, 15, 0x0F, 0x1B, 0x18, 0x0B, 0x0E, 0x09, 0x47, 0x94, 0x35, 0x0A, 0x13, 0x05, 0x08, 0x03, 0x00, 
			0xE1, 15, 0x0F, 0x3A, 0x37, 0x0B, 0x0C, 0x05, 0x4A, 0x24, 0x39, 0x07, 0x10, 0x04, 0x27, 0x25, 0x00, 
#elif GAMMA9486 == 3
            // 2.2 HSD 3.5 Inch Initial Code not bad
			0xE0, 15, 0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98, 0x37, 0x0A, 0x13, 0x04, 0x11, 0x0D, 0x00, 
			0xE1, 15, 0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75, 0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00, 
#elif GAMMA9486 == 4
            // 3.2 TM  3.2 Inch Initial Code not bad
			0xE0, 15, 0x0F, 0x21, 0x1C, 0x0B, 0x0E, 0x08, 0x49, 0x98, 0x38, 0x09, 0x11, 0x03, 0x14, 0x10, 0x00, 
			0xE1, 15, 0x0F, 0x2F, 0x2B, 0x0C, 0x0E, 0x06, 0x47, 0x76, 0x37, 0x07, 0x11, 0x04, 0x23, 0x1E, 0x00, 
#elif GAMMA9486 == 5
            // 4.2 WTK 3.5 Inch Initial Code too white
			0xE0, 15, 0x0F, 0x10, 0x08, 0x05, 0x09, 0x05, 0x37, 0x98, 0x26, 0x07, 0x0F, 0x02, 0x09, 0x07, 0x00, 
			0xE1, 15, 0x0F, 0x38, 0x36, 0x0D, 0x10, 0x08, 0x59, 0x76, 0x48, 0x0A, 0x16, 0x0A, 0x37, 0x2F, 0x00, 
#endif
        };
        table8_ads = ILI9486_regValues, table_size = sizeof(ILI9486_regValues);
        p16 = (int16_t *) & HEIGHT;
        *p16 = 480;
        p16 = (int16_t *) & WIDTH;
        *p16 = 320;

    }
    _lcd_rev = ((_lcd_capable & REV_SCREEN) != 0);
    if (table8_ads != NULL) {
        static const uint8_t reset_off[] PROGMEM = {
            0x01, 0,            //Soft Reset
            TFTLCD_DELAY8, 150,  // .kbv will power up with ONLY reset, sleep out, display on
            0x28, 0,            //Display Off
            0x3A, 1, 0x55,      //Pixel read=565, write=565.
        };
        static const uint8_t wake_on[] PROGMEM = {
			0x11, 0,            //Sleep Out
            TFTLCD_DELAY8, 150,
            0x29, 0,            //Display On
        };
		init_table(&reset_off, sizeof(reset_off));
	    init_table(table8_ads, table_size);   //can change PIXFMT
		init_table(&wake_on, sizeof(wake_on));
    }
    setRotation(0);             //PORTRAIT
    invertDisplay(false);

}
