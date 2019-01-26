//#define USE_SPECIAL             //check for custom drivers
#include <Arduino.h>
#define WR_ACTIVE2  {WR_ACTIVE; WR_ACTIVE;}
#define WR_ACTIVE4  {WR_ACTIVE2; WR_ACTIVE2;}
#define WR_ACTIVE8  {WR_ACTIVE4; WR_ACTIVE4;}
#define RD_ACTIVE2  {RD_ACTIVE; RD_ACTIVE;}
#define RD_ACTIVE4  {RD_ACTIVE2; RD_ACTIVE2;}
#define RD_ACTIVE8  {RD_ACTIVE4; RD_ACTIVE4;}
#define RD_ACTIVE16 {RD_ACTIVE8; RD_ACTIVE8;}
#define WR_IDLE2  {WR_IDLE; WR_IDLE;}
#define WR_IDLE4  {WR_IDLE2; WR_IDLE2;}

#if defined(USE_SPECIAL)
//#include "mcufriend_special.h"
#if !defined(USE_SPECIAL_FAIL)
#warning WE ARE USING A SPECIAL CUSTOM DRIVER
#endif
#endif
#if !defined(USE_SPECIAL) || defined (USE_SPECIAL_FAIL)

#if 0
//################################### UNO ##############################
#elif defined(__AVR_ATmega328P__)       //regular UNO shield on UNO
#define RD_PORT PORTC
#define RD_PIN  0
#define WR_PORT PORTC
#define WR_PIN  1
#define CD_PORT PORTC
#define CD_PIN  2
#define CS_PORT PORTC
#define CS_PIN  3
#define RESET_PORT PORTC
#define RESET_PIN  4

#define BMASK         0x03              //more intuitive style for mixed Ports
#define DMASK         0xFC              //does exactly the same as previous
#define write_8(x)    { PORTB = (PORTB & ~BMASK) | ((x) & BMASK); PORTD = (PORTD & ~DMASK) | ((x) & DMASK); }
#define read_8()      ( (PINB & BMASK) | (PIND & DMASK) )
#define setWriteDir() { DDRB |=  BMASK; DDRD |=  DMASK; }
#define setReadDir()  { DDRB &= ~BMASK; DDRD &= ~DMASK; }
#define write8(x)     { write_8(x); WR_STROBE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; dst = read_8(); RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))




//####################################### STM32 ############################
// NUCLEO:   ARDUINO_NUCLEO_xxxx from ST Core or ARDUINO_STM_NUCLEO_F103RB from MapleCore
// BLUEPILL: ARDUINO_NUCLEO_F103C8 / ARDUINO_BLUEPILL_F103C8 from ST Core or ARDUINO_GENERIC_STM32F103C from MapleCore
// MAPLE_REV3: n/a from ST Core or ARDUINO_MAPLE_REV3 from MapleCore
// ST Core:   ARDUINO_ARCH_STM32
// MapleCore: __STM32F1__
#elif defined(__STM32F1__) || defined(ARDUINO_ARCH_STM32)   //MapleCore or ST Core
#define IS_NUCLEO64 ( defined(ARDUINO_STM_NUCLEO_F103RB) \
                   || defined(ARDUINO_NUCLEO_F030R8) || defined(ARDUINO_NUCLEO_F091RC) \
                   || defined(ARDUINO_NUCLEO_F103RB) || defined(ARDUINO_NUCLEO_F303RE) \
                   || defined(ARDUINO_NUCLEO_F401RE) || defined(ARDUINO_NUCLEO_F411RE) \
                   || defined(ARDUINO_NUCLEO_F446RE) || defined(ARDUINO_NUCLEO_L053R8) \
                   || defined(ARDUINO_NUCLEO_L152RE) || defined(ARDUINO_NUCLEO_L476RG) \
                    )
// F1xx, F4xx, L4xx have different registers and styles.  General Macros
#if defined(__STM32F1__)   //weird Maple Core
#define REGS(x) regs->x
#else                      //regular ST Core
#define REGS(x) x
#endif
#define PIN_HIGH(port, pin)   (port)-> REGS(BSRR) = (1<<(pin))
#define PIN_LOW(port, pin)    (port)-> REGS(BSRR) = (1<<((pin)+16))
#define PIN_MODE2(reg, pin, mode) reg=(reg&~(0x3<<((pin)<<1)))|(mode<<((pin)<<1))
#define GROUP_MODE(port, reg, mask, val)  {port->REGS(reg) = (port->REGS(reg) & ~(mask)) | ((mask)&(val)); }

// Family specific Macros.  F103 needs ST and Maple compatibility
// note that ILI9320 class of controller has much slower Read cycles
#if 0
#elif defined(__STM32F1__) || defined(ARDUINO_NUCLEO_F103C8) || defined(ARDUINO_BLUEPILL_F103C8) || defined(ARDUINO_NUCLEO_F103RB)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#if defined(__STM32F1__)  //MapleCore crts.o does RCC.  not understand regular syntax anyway
#define GPIO_INIT()      
#else
#define GPIO_INIT()   { RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN; \
        AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;}
#endif
#define GP_OUT(port, reg, mask)           GROUP_MODE(port, reg, mask, 0x33333333)
#define GP_INP(port, reg, mask)           GROUP_MODE(port, reg, mask, 0x44444444)
#define PIN_OUTPUT(port, pin) {\
        if (pin < 8) {GP_OUT(port, CRL, 0xF<<((pin)<<2));} \
        else {GP_OUT(port, CRH, 0xF<<((pin&7)<<2));} \
    }
#define PIN_INPUT(port, pin) { \
        if (pin < 8) { GP_INP(port, CRL, 0xF<<((pin)<<2)); } \
        else { GP_INP(port, CRH, 0xF<<((pin&7)<<2)); } \
    }

#else
#error unsupported STM32
#endif

#if 0
#elif defined(ARDUINO_GENERIC_STM32F103C) || defined(ARDUINO_NUCLEO_F103C8) || defined(ARDUINO_BLUEPILL_F103C8)
#warning Uno Shield on BLUEPILL
#define RD_PORT GPIOB
//#define RD_PIN  5
#define RD_PIN  0  //hardware mod to Adapter.  Allows use of PB5 for SD Card
#define WR_PORT GPIOB
#define WR_PIN  6
#define CD_PORT GPIOB
#define CD_PIN  7
#define CS_PORT GPIOB
#define CS_PIN  8
#define RESET_PORT GPIOB
#define RESET_PIN  9

// configure macros for the data pins
#define write_8(d)    { GPIOA->REGS(BSRR) = 0x00FF << 16; GPIOA->REGS(BSRR) = (d) & 0xFF; }
#define read_8()      (GPIOA->REGS(IDR) & 0xFF)
//                                         PA7 ..PA0
#define setWriteDir() {GP_OUT(GPIOA, CRL, 0xFFFFFFFF); }
#define setReadDir()  {GP_INP(GPIOA, CRL, 0xFFFFFFFF); }


// configure macros for the data pins
#define write_8(d) { \
        GPIOA->REGS(BSRR) = 0x0700 << 16; \
        GPIOB->REGS(BSRR) = 0x0438 << 16; \
        GPIOC->REGS(BSRR) = 0x0080 << 16; \
        GPIOA->REGS(BSRR) = (  ((d) & (1<<0)) << 9) \
                            | (((d) & (1<<2)) << 8) \
                            | (((d) & (1<<7)) << 1); \
        GPIOB->REGS(BSRR) = (  ((d) & (1<<3)) << 0) \
                            | (((d) & (1<<4)) << 1) \
                            | (((d) & (1<<5)) >> 1) \
                            | (((d) & (1<<6)) << 4); \
        GPIOC->REGS(BSRR) = (  ((d) & (1<<1)) << 6); \
    }

#define read_8() (       (  (  (GPIOA->REGS(IDR) & (1<<9)) >> 9) \
                            | ((GPIOC->REGS(IDR) & (1<<7)) >> 6) \
                            | ((GPIOA->REGS(IDR) & (1<<10)) >> 8) \
                            | ((GPIOB->REGS(IDR) & (1<<3)) >> 0) \
                            | ((GPIOB->REGS(IDR) & (1<<5)) >> 1) \
                            | ((GPIOB->REGS(IDR) & (1<<4)) << 1) \
                            | ((GPIOB->REGS(IDR) & (1<<10)) >> 4) \
                            | ((GPIOA->REGS(IDR) & (1<<8))  >> 1)))




#else
#error REGS group
#endif

#ifndef IDLE_DELAY
#define IDLE_DELAY    { WR_IDLE; }
#endif

#define write8(x)     { write_8(x); WRITE_DELAY; WR_STROBE; IDLE_DELAY; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; READ_DELAY; dst = read_8(); RD_IDLE; RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

//################################### ESP32 ##############################
#elif defined(ESP32)       //regular UNO shield on TTGO D1 R32 (ESP32)
#define LCD_RD  2  //LED
#define LCD_WR  4
#define LCD_RS 15  //hard-wired to A2 (GPIO35) 
#define LCD_CS 33  //hard-wired to A3 (GPIO34)
#define LCD_RST 32 //hard-wired to A4 (GPIO36)

#define LCD_D0 12
#define LCD_D1 13
#define LCD_D2 26
#define LCD_D3 25
#define LCD_D4 17
#define LCD_D5 16
#define LCD_D6 27
#define LCD_D7 14

#define RD_PORT GPIO.out
#define RD_PIN  LCD_RD
#define WR_PORT GPIO.out
#define WR_PIN  LCD_WR
#define CD_PORT GPIO.out
#define CD_PIN  LCD_RS
#define CS_PORT GPIO.out1.val
#define CS_PIN  LCD_CS
#define RESET_PORT GPIO.out1.val
#define RESET_PIN  LCD_RST

static inline uint32_t map_8(uint32_t d)
{
    return (
               0
               | ((d & (1 << 0)) << (LCD_D0 - 0))
               | ((d & (1 << 1)) << (LCD_D1 - 1))
               | ((d & (1 << 2)) << (LCD_D2 - 2))
               | ((d & (1 << 3)) << (LCD_D3 - 3))
               | ((d & (1 << 4)) << (LCD_D4 - 4))
               | ((d & (1 << 5)) << (LCD_D5 - 5))
               | ((d & (1 << 6)) << (LCD_D6 - 6))
               | ((d & (1 << 7)) << (LCD_D7 - 7))
           );
}

static inline uint8_t map_32(uint32_t d)
{
    return (
               0
               | ((d & (1 << LCD_D0)) >> (LCD_D0 - 0))
               | ((d & (1 << LCD_D1)) >> (LCD_D1 - 1))
               | ((d & (1 << LCD_D2)) >> (LCD_D2 - 2))
               | ((d & (1 << LCD_D3)) >> (LCD_D3 - 3))
               | ((d & (1 << LCD_D4)) >> (LCD_D4 - 4))
               | ((d & (1 << LCD_D5)) >> (LCD_D5 - 5))
               | ((d & (1 << LCD_D6)) >> (LCD_D6 - 6))
               | ((d & (1 << LCD_D7)) >> (LCD_D7 - 7))
           );
}

static inline void write_8(uint16_t data)
{
    GPIO.out_w1tc = map_8(0xFF);  //could define once as DMASK
    GPIO.out_w1ts = map_8(data);
}

static inline uint8_t read_8()
{
    return map_32(GPIO.in);
}
static void setWriteDir()
{
    pinMode(LCD_D0, OUTPUT);
    pinMode(LCD_D1, OUTPUT);
    pinMode(LCD_D2, OUTPUT);
    pinMode(LCD_D3, OUTPUT);
    pinMode(LCD_D4, OUTPUT);
    pinMode(LCD_D5, OUTPUT);
    pinMode(LCD_D6, OUTPUT);
    pinMode(LCD_D7, OUTPUT);
}

static void setReadDir()
{
    pinMode(LCD_D0, INPUT);
    pinMode(LCD_D1, INPUT);
    pinMode(LCD_D2, INPUT);
    pinMode(LCD_D3, INPUT);
    pinMode(LCD_D4, INPUT);
    pinMode(LCD_D5, INPUT);
    pinMode(LCD_D6, INPUT);
    pinMode(LCD_D7, INPUT);
}

#define WRITE_DELAY { }
#define READ_DELAY  { }

#define write8(x)     { write_8(x); WRITE_DELAY; WR_STROBE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; READ_DELAY; dst = read_8(); RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

#define PIN_LOW(p, b)        (digitalWrite(b, LOW))
#define PIN_HIGH(p, b)       (digitalWrite(b, HIGH))
#define PIN_OUTPUT(p, b)     (pinMode(b, OUTPUT))

#else
#error MCU unsupported
#endif                          // regular UNO shields on Arduino boards

#endif                          //!defined(USE_SPECIAL) || defined (USE_SPECIAL_FAIL)

#define RD_ACTIVE  PIN_LOW(RD_PORT, RD_PIN)
#define RD_IDLE    PIN_HIGH(RD_PORT, RD_PIN)
#define RD_OUTPUT  PIN_OUTPUT(RD_PORT, RD_PIN)
#define WR_ACTIVE  PIN_LOW(WR_PORT, WR_PIN)
#define WR_IDLE    PIN_HIGH(WR_PORT, WR_PIN)
#define WR_OUTPUT  PIN_OUTPUT(WR_PORT, WR_PIN)
#define CD_COMMAND PIN_LOW(CD_PORT, CD_PIN)
#define CD_DATA    PIN_HIGH(CD_PORT, CD_PIN)
#define CD_OUTPUT  PIN_OUTPUT(CD_PORT, CD_PIN)
#define CS_ACTIVE  PIN_LOW(CS_PORT, CS_PIN)
#define CS_IDLE    PIN_HIGH(CS_PORT, CS_PIN)
#define CS_OUTPUT  PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)
#define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)

 // General macros.   IOCLR registers are 1 cycle when optimised.
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }       //PWLW=TWRL=50ns
#define RD_STROBE RD_IDLE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE      //PWLR=TRDL=150ns, tDDR=100ns

#if !defined(GPIO_INIT)
#define GPIO_INIT()
#endif
#define CTL_INIT()   { GPIO_INIT(); RD_OUTPUT; WR_OUTPUT; CD_OUTPUT; CS_OUTPUT; RESET_OUTPUT; }
#define WriteCmd(x)  { CD_COMMAND; write16(x); CD_DATA; }
#define WriteData(x) { write16(x); }
