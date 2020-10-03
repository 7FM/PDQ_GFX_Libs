// This is the PDQ re-mixed version of Adafruit's library
// here is the original copyright notice:

/***************************************************
  This is an Arduino Library for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

//===============================================================
// This PDQ optimized version is by Xark
//
// Inspiration from Paul Stoffregen and the Teensy 3.1 community.
//
// GOALS:
//  1) Maintain "sketch" compatibility with original Adafruit libraries.
//  2) Be as much faster as is reasonably possible honoring goal 1. :-)
//  3) Be at least as small as Adafruit libraries.
//
// I believe all three of these have largely been achieved:
// 1) Near full compatibility.  Only minor initialization changes in original sketch.
// 2) Between ~2.5 and ~12 times faster (fillRect ~2.5x, drawLine ~12x).
// An average of ~4x faster over entire "graphictest.ino" benchmark.
//
// Even if this library is faster, it was based on the Adafruit original.
// Adafruit deserves your support for making their library open-source (and
// for having some nice LCD modules and all kinds of other great parts too).
// Consider giving them your support if possible!

#if !defined(_PDQ_ILI9341H_)
#define _PDQ_ILI9341H_

#include "Arduino.h"
#include "Print.h"

#include <PDQ_GFX/PDQ_GFX.h>

#include <avr/pgmspace.h>

#include <FastPin.h>

#ifdef AVR_HARDWARE_SPI
#include <SPI.h>
#endif

#if !defined(__AVR_ATtiny85__) && !defined(__AVR_ATtiny45__)
#define INLINE inline
#define INLINE_OPT __attribute__((always_inline))
#else
#define INLINE
#define INLINE_OPT
#endif

// Color definitions
enum : color_t {
    ILI9341_BLACK = 0x0000,
    ILI9341_BLUE = 0x001F,
    ILI9341_RED = 0xF800,
    ILI9341_GREEN = 0x07E0,
    ILI9341_CYAN = 0x07FF,
    ILI9341_MAGENTA = 0xF81F,
    ILI9341_YELLOW = 0xFFE0,
    ILI9341_WHITE = 0xFFFF,
};

// ILI9341 commands
// For datasheet see https://www.adafruit.com/products/1480
typedef enum : uint8_t {
    ILI9341_NOP = 0x00,
    ILI9341_SWRESET = 0x01,
    ILI9341_RDDID = 0x04,
    ILI9341_RDDST = 0x09,

    ILI9341_SLPIN = 0x10,
    ILI9341_SLPOUT = 0x11,
    ILI9341_PTLON = 0x12,
    ILI9341_NORON = 0x13,

    ILI9341_RDMODE = 0x0A,
    ILI9341_RDMADCTL = 0x0B,
    ILI9341_RDPIXFMT = 0x0C,
    ILI9341_RDIMGFMT = 0x0A,
    ILI9341_RDSELFDIAG = 0x0F,

    ILI9341_INVOFF = 0x20,
    ILI9341_INVON = 0x21,
    ILI9341_GAMMASET = 0x26,
    ILI9341_DISPOFF = 0x28,
    ILI9341_DISPON = 0x29,

    ILI9341_CASET = 0x2A,
    ILI9341_PASET = 0x2B,
    ILI9341_RAMWR = 0x2C,
    ILI9341_RAMRD = 0x2E,

    ILI9341_PTLAR = 0x30,
    ILI9341_MADCTL = 0x36,
    ILI9341_PIXFMT = 0x3A,

    ILI9341_FRMCTR1 = 0xB1,
    ILI9341_FRMCTR2 = 0xB2,
    ILI9341_FRMCTR3 = 0xB3,
    ILI9341_INVCTR = 0xB4,
    ILI9341_DFUNCTR = 0xB6,

    ILI9341_PWCTR1 = 0xC0,
    ILI9341_PWCTR2 = 0xC1,
    ILI9341_PWCTR3 = 0xC2,
    ILI9341_PWCTR4 = 0xC3,
    ILI9341_PWCTR5 = 0xC4,
    ILI9341_VMCTR1 = 0xC5,
    ILI9341_VMCTR2 = 0xC7,

    ILI9341_RDID1 = 0xDA,
    ILI9341_RDID2 = 0xDB,
    ILI9341_RDID3 = 0xDC,
    ILI9341_RDID4 = 0xDD,

    ILI9341_GMCTRP1 = 0xE0,
    ILI9341_GMCTRN1 = 0xE1,

    // ILI9341_PWCTR6	= 0xFC,
} ILI9341_Commands;

// some other misc. constants
enum {
    // screen dimensions
    ILI9341_TFTWIDTH = 240,
    ILI9341_TFTHEIGHT = 320,

    // MADCTL bits
    ILI9341_MADCTL_MH = 0x04,  // bit 2 = 0 for refresh left -> right, 1 for refresh right -> left
    ILI9341_MADCTL_RGB = 0x00, // bit 3 = 0 for RGB color order
    ILI9341_MADCTL_BGR = 0x08, // bit 3 = 1 for BGR color order
    ILI9341_MADCTL_ML = 0x10,  // bit 4 = 0 for refresh top -> bottom, 1 for bottom -> top
    ILI9341_MADCTL_MV = 0x20,  // bit 5 = 0 for column, row order (portrait), 1 for row, column order (landscape)
    ILI9341_MADCTL_MX = 0x40,  // bit 6 = 0 for left -> right, 1 for right -> left order
    ILI9341_MADCTL_MY = 0x80,  // bit 7 = 0 for top -> bottom, 1 for bottom -> top

    // delay indicator bit for commandList()
    DELAY = 0x80
};

#if defined(AVR_HARDWARE_SPI)
extern "C" {
// 10 cycle delay (including "rcall")
__attribute__((noinline)) __attribute__((naked)) __attribute__((used)) void delay10() {
    __asm__ __volatile__(
    // +3 (rcall to get here)
#if !defined(__AVR_HAVE_RAMPD__)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
        "	nop\n"           // +1 (1-cycle NOP)
#else
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
#endif
        "	ret\n" // +4 (or +5 on >64KB AVR with RAMPD reg)
                   // = 10 cycles
        :
        :
        :);
}

// 13 cycle delay (including "call")
__attribute__((noinline)) __attribute__((naked)) __attribute__((used)) void delay13() {
    __asm__ __volatile__(
        // +4 (call to get here)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
#if !defined(__AVR_HAVE_RAMPD__)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
#else
        "	nop\n"           // +1 (1-cycle NOP)
#endif
        "	ret\n" // +4 (or +5 on >64KB AVR with RAMPD reg)
                   // = 13 cycles
        :
        :
        :);
}

// 15 cycle delay (including "rcall")
__attribute__((noinline)) __attribute__((naked)) __attribute__((used)) void delay15() {
    __asm__ __volatile__(
        // +3 (rcall to get here)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
#if !defined(__AVR_HAVE_RAMPD__)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
#else
        "	nop\n"           // +1 (1-cycle NOP)
#endif
        "	ret\n" // +4 (or +5 on >64KB AVR with RAMPD reg)
                   // = 15 cycles
        :
        :
        :);
}

// 17 cycle delay (including "rcall")
__attribute__((noinline)) __attribute__((naked)) __attribute__((used)) void delay17() {
    __asm__ __volatile__(
        // +3 (rcall to get here)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
#if !defined(__AVR_HAVE_RAMPD__)
        "	adiw	r24,0\n" // +2 (2-cycle NOP)
#else
        "	nop\n"           // +1 (2-cycle NOP)
#endif
        "	ret\n" // +4 (or +5 on >64KB AVR with RAMPD reg)
                   // = 17 cycles
        :
        :
        :);
}
}

// normal SPI write with minimal hand-tuned delay (assuming max DIV2 SPI rate)
static INLINE INLINE_OPT void spiWrite(uint8_t data) {
    SPDR = data;
    __asm__ __volatile__(
        "	rcall	delay17\n" // call mangled delay17 (compiler would needlessly save/restore regs)
        :
        :
        :);
}

// special SPI write with minimal hand-tuned delay (assuming max DIV2 SPI rate) - minus 2 cycles for RS (etc.) change
static INLINE INLINE_OPT void spiWrite_preCmd(uint8_t data) {
    SPDR = data;

    __asm__ __volatile__(
        "	rcall	delay15\n" // call mangled delay15 (compiler would needlessly save/restore regs)
        :
        :
        :);
}

// SPI 16-bit write with minimal hand-tuned delay (assuming max DIV2 SPI rate)
static INLINE INLINE_OPT void spiWrite16(uint16_t data) {
    uint8_t temp;
    __asm__ __volatile__(
        "	out	%[spi],%[hi]\n" // write SPI data (18 cycles until next write)
        "	rcall	delay17\n"  // call mangled delay17 (compiler would needlessly save/restore regs)
        "	out	%[spi],%[lo]\n" // write SPI data (18 cycles until next write)
        "	rcall	delay17\n"  // call mangled delay17 (compiler would needlessly save/restore regs)

        : [ temp ] "=d"(temp)
        : [ spi ] "i"(_SFR_IO_ADDR(SPDR)), [ lo ] "r"((uint8_t)data), [ hi ] "r"((uint8_t)(data >> 8))
        :);
}

// SPI 16-bit write with minimal hand-tuned delay (assuming max DIV2 SPI rate) minus 2 cycles
static INLINE INLINE_OPT void spiWrite16_preCmd(uint16_t data) {
    uint8_t temp;
    __asm__ __volatile__(
        "	out	%[spi],%[hi]\n" // write SPI data (18 cycles until next write)
        "	rcall	delay17\n"  // call mangled delay17 (compiler would needlessly save/restore regs)
        "	out	%[spi],%[lo]\n" // write SPI data (18 cycles until next write)
        "	rcall	delay15\n"  // call mangled delay15 (compiler would needlessly save/restore regs)

        : [ temp ] "=d"(temp)
        : [ spi ] "i"(_SFR_IO_ADDR(SPDR)), [ lo ] "r"((uint8_t)data), [ hi ] "r"((uint8_t)(data >> 8))
        :);
}

// SPI 16-bit write with minimal hand-tuned delay (assuming max DIV2 SPI rate) minus 4 cycles
static INLINE INLINE_OPT void spiWrite16_lineDraw(uint16_t data) {
    uint8_t temp;
    __asm__ __volatile__(
        "	out	%[spi],%[hi]\n" // write SPI data (18 cycles until next write)
        "	rcall	delay17\n"  // call mangled delay17 (compiler would needlessly save/restore regs)
        "	out	%[spi],%[lo]\n" // write SPI data (18 cycles until next write)

        : [ temp ] "=d"(temp)
        : [ spi ] "i"(_SFR_IO_ADDR(SPDR)), [ lo ] "r"((uint8_t)data), [ hi ] "r"((uint8_t)(data >> 8))
        :);
}

// normal SPI write with minimal hand-tuned delay (assuming max DIV2 SPI rate)
static INLINE INLINE_OPT void spiWrite16(uint16_t data, int count) {
    uint8_t temp;
    __asm__ __volatile__(
        "	sbiw	%[count],0\n" // test count
        "	brmi	4f\n"         // if < 0 then done
        "	breq	4f\n"         // if == 0 then done
        "1:	out	%[spi],%[hi]\n"   // write SPI data (18 cycles until next write)
        "	rcall	delay17\n"    // call mangled delay17 (compiler would needlessly save/restore regs)
        "	out	%[spi],%[lo]\n"   // write SPI data (18 cycles until next write)
        "	rcall	delay13\n"    // call mangled delay13 (compiler would needlessly save/restore regs)
        "	sbiw	%[count],1\n" // +2	decrement count
        "	brne	1b\n"         // +2/1	if != 0 then loop
                                  // = 13 + 2 + 2 (17 cycles)
        "4:\n"

        : [ temp ] "=d"(temp), [ count ] "+w"(count)
        : [ spi ] "i"(_SFR_IO_ADDR(SPDR)), [ lo ] "r"((uint8_t)data), [ hi ] "r"((uint8_t)(data >> 8))
        :);
}

#else // bit-bang
#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__)
// USI hardware assisted
static __attribute__((noinline)) void spiWrite(uint8_t data) {
    USIDR = data;
    __asm__ __volatile__(
        "	out %[spi],%[clkp0]\n" // MSB
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n" // LSB
        "	out %[spi],%[clkp1]\n"
        :
        : [ spi ] "i"(_SFR_IO_ADDR(USICR)), [ clkp0 ] "a"((uint8_t)((1 << USIWM0) | (0 << USICS0) | (1 << USITC))), [ clkp1 ] "a"((uint8_t)((1 << USIWM0) | (0 << USICS0) | (1 << USITC) | (1 << USICLK)))
        :);
}
static __attribute__((noinline)) void spiWrite16(uint16_t data) {
    USIDR = data >> 8;
    __asm__ __volatile__(
        "	out %[spi],%[clkp0]\n" // MSB
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n" // LSB
        "	out %[spi],%[clkp1]\n"
        :
        : [ spi ] "i"(_SFR_IO_ADDR(USICR)), [ clkp0 ] "a"((uint8_t)((1 << USIWM0) | (0 << USICS0) | (1 << USITC))), [ clkp1 ] "a"((uint8_t)((1 << USIWM0) | (0 << USICS0) | (1 << USITC) | (1 << USICLK)))
        :);

    USIDR = data & 0xff;
    __asm__ __volatile__(
        "	out %[spi],%[clkp0]\n" // MSB
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n"
        "	out %[spi],%[clkp1]\n"
        "	out %[spi],%[clkp0]\n" // LSB
        "	out %[spi],%[clkp1]\n"
        :
        : [ spi ] "i"(_SFR_IO_ADDR(USICR)), [ clkp0 ] "a"((uint8_t)((1 << USIWM0) | (0 << USICS0) | (1 << USITC))), [ clkp1 ] "a"((uint8_t)((1 << USIWM0) | (0 << USICS0) | (1 << USITC) | (1 << USICLK)))
        :);
}
#else
static __attribute__((noinline)) void spiWrite(uint8_t data) {
    // Fast SPI bitbang swiped from LPD8806 library
    for (uint8_t bit = 0x80; bit; bit >>= 1) {
        if (data & bit)
            FastPin<ILI9341_MOSI_PIN>::hi();
        else
            FastPin<ILI9341_MOSI_PIN>::lo();

        FastPin<ILI9341_SCLK_PIN>::hi();
        FastPin<ILI9341_SCLK_PIN>::lo();
    }
}
static __attribute__((noinline)) void spiWrite16(uint16_t data) {
    spiWrite(data >> 8);
    spiWrite(data & 0xff);
}
#endif
static INLINE INLINE_OPT void spiWrite_preCmd(uint8_t data) {
    spiWrite(data);
}
static INLINE INLINE_OPT void spiWrite16_preCmd(uint16_t data) {
    spiWrite16(data);
}
static INLINE INLINE_OPT void spiWrite16_lineDraw(uint16_t data) {
    spiWrite16(data);
}
static INLINE INLINE_OPT void spiWrite16(uint16_t data, int count) {
    while (count-- > 0)
        spiWrite16(data);
}
static inline void delay10() {}
static inline void delay13() {}
static inline void delay15() {}
static inline void delay17() {}
#endif

#if defined(AVR_HARDWARE_SPI)
// Ugly... I know but it works and you cant have more than one display either with this library
// May the compiler remove optimize it away if unused x.x
static volatile uint8_t save_SPCR; // initial SPCR value/saved SPCR value (swapped in spi_begin/spi_end)
static volatile uint8_t save_SPSR; // initial SPSR value/saved SPSR value (swapped in spi_begin/spi_end)
#endif

typedef uint16_t coord_t;
typedef int16_t s_coord_t;

#if defined(AVR_HARDWARE_SPI)
#define _TEMPLATE_CLASS_DEF template <uint8_t ILI9341_CS_PIN, uint8_t ILI9341_DC_PIN, uint8_t ILI9341_RST_PIN, bool ILI9341_SAVE_SPI_SETTINGS, bool ILI9341_RESTORE_PREV_SPI_SETTINGS = false>
#define _TEMPLATE_DEF template <uint8_t ILI9341_CS_PIN, uint8_t ILI9341_DC_PIN, uint8_t ILI9341_RST_PIN, bool ILI9341_SAVE_SPI_SETTINGS, bool ILI9341_RESTORE_PREV_SPI_SETTINGS>

#define _TEMPLATE_CLASS PDQ_ILI9341<ILI9341_CS_PIN, ILI9341_DC_PIN, ILI9341_RST_PIN, ILI9341_SAVE_SPI_SETTINGS, ILI9341_RESTORE_PREV_SPI_SETTINGS>
#else
#define _TEMPLATE_CLASS_DEF template <uint8_t ILI9341_CS_PIN, uint8_t ILI9341_DC_PIN, uint8_t ILI9341_RST_PIN>
#define _TEMPLATE_DEF template <uint8_t ILI9341_CS_PIN, uint8_t ILI9341_DC_PIN, uint8_t ILI9341_RST_PIN>

#define _TEMPLATE_CLASS PDQ_ILI9341<ILI9341_CS_PIN, ILI9341_DC_PIN, ILI9341_RST_PIN>
#endif

#define _PARENT PDQ_GFX<_TEMPLATE_CLASS, coord_t, s_coord_t, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT>

_TEMPLATE_CLASS_DEF
class PDQ_ILI9341 : public _PARENT {
  public:
    // higher-level routines
    static void inline begin();
    static void setAddrWindow(coord_t x0, coord_t y0, coord_t x1, coord_t y1);
    static void pushColor(color_t color);
    static void pushColor(color_t color, int cnt);

    // Pass 8-bit (each) R,G,B, get back 16-bit packed color
    static constexpr INLINE color_t color565(uint8_t r, uint8_t g, uint8_t b) {
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }

    // required driver primitive methods (all except drawPixel can call generic version in PDQ_GFX with "_" postfix).
    static void drawPixel(coord_t x, coord_t y, color_t color);
    static void drawFastVLine(coord_t x, s_coord_t y, coord_t h, color_t color);
    static void drawFastHLine(s_coord_t x, coord_t y, coord_t w, color_t color);
    static void setRotation(uint8_t r);
    static void invertDisplay(boolean i);

    static inline void fillScreen(color_t color) __attribute__((always_inline)) {
        _PARENT::fillScreen_(color); // call generic version
    }

    static void drawLine(coord_t x0, coord_t y0, coord_t x1, coord_t y1, color_t color);
    static void fillRect(s_coord_t x, s_coord_t y, coord_t w, coord_t h, color_t color);

    // === lower-level internal routines =========
    static void commandList(const uint8_t *addr);

    // NOTE: Make sure each spi_begin() is matched with a single spi_end() (and don't call either twice)
    // set CS back to low (LCD selected)
    static inline void spi_begin() __attribute__((always_inline)) {
#if defined(AVR_HARDWARE_SPI)
        if (ILI9341_SAVE_SPI_SETTINGS) {
            if (ILI9341_RESTORE_PREV_SPI_SETTINGS) {
                swapValue(save_SPCR, SPCR); // swap initial/current SPCR settings
                uint8_t tmp = SPSR;
                SPSR = save_SPSR & 0x1;
                save_SPSR = tmp; // swap initial/current SPSR settings
            } else {
                SPCR = save_SPCR;
                SPSR = save_SPSR & 0x1; // SPI2x mask
            }
        }
#endif
        FastPin<ILI9341_CS_PIN>::lo(); // CS <= LOW (selected)
    }

    // NOTE: Make sure each spi_begin() is matched with a single spi_end() (and don't call either twice)
    // reset CS back to high (LCD unselected)
    static inline void spi_end() __attribute__((always_inline)) {
        FastPin<ILI9341_CS_PIN>::hi(); // CS <= HIGH (deselected)
#if defined(AVR_HARDWARE_SPI)
        if (ILI9341_SAVE_SPI_SETTINGS && ILI9341_RESTORE_PREV_SPI_SETTINGS) {
            swapValue(SPCR, save_SPCR); // swap current/initial SPCR settings
            uint8_t tmp = SPSR;
            SPSR = save_SPSR & 0x1;
            save_SPSR = tmp; // swap initial/current SPSR settings
        }
#endif
    }

    // write SPI byte with RS (aka D/C) pin set low to indicate a command byte (and then reset back to high when done)
    static INLINE void writeCommand(uint8_t data) INLINE_OPT {
        FastPin<ILI9341_DC_PIN>::lo(); // RS <= LOW indicate command byte
        spiWrite_preCmd(data);
        FastPin<ILI9341_DC_PIN>::hi(); // RS <= HIGH indicate data byte (always assumed left in data mode)
    }

    // write SPI byte with RS assumed low indicating a data byte
    static inline void writeData(uint8_t data) __attribute__((always_inline)) {
        spiWrite(data);
    }

    // internal version that does not spi_begin()/spi_end()
    static INLINE void setAddrWindow_(coord_t x0, coord_t y0, coord_t x1, coord_t y1) INLINE_OPT {
        writeCommand(ILI9341_CASET); // column address set
        spiWrite16(x0);              // XSTART
        spiWrite16_preCmd(x1);       // XEND
        writeCommand(ILI9341_PASET); // row address set
        spiWrite16(y0);              // YSTART
        spiWrite16_preCmd(y1);       // YEND
        writeCommand(ILI9341_RAMWR); // write to RAM
    }

    typedef PDQ_GFX_Button_<_TEMPLATE_CLASS, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT> PDQ_GFX_Button;
};

/***************************************************
  This is an Arduino Library for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
_TEMPLATE_DEF
void _TEMPLATE_CLASS::commandList(const uint8_t *addr) {
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = pgm_read_byte(addr++); // Number of commands to follow
    while (numCommands--)                // For each command...
    {
        writeCommand(pgm_read_byte(addr++)); // Read, issue command
        numArgs = pgm_read_byte(addr++);     // Number of args to follow
        ms = numArgs & DELAY;                // If hibit set, delay follows args
        numArgs &= ~DELAY;                   // Mask out delay bit
        while (numArgs--)                    // For each argument...
        {
            writeData(pgm_read_byte(addr++)); // Read, issue argument
        }

        if (ms) {
            ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
            if (ms == 255)
                ms = 500; // If 255, delay for 500 ms
            delay(ms);
        }
    }
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::begin(void) {

    // set CS and RS pin directions to output
    FastPin<ILI9341_CS_PIN>::setOutput();
    FastPin<ILI9341_DC_PIN>::setOutput();
#if !defined(AVR_HARDWARE_SPI)
#if defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    USICR = (0 << USISIE) | (0 << USIOIE) | (0 << USIWM1) | (1 << USIWM0) | (0 << USICS1) | (1 << USICS0) | (1 << USICLK) | (0 << USITC);
#endif
#if defined(ILI9341_MISO_PIN)
    FastPin<ILI9341_MISO_PIN>::setInput();
#endif
    FastPin<ILI9341_MOSI_PIN>::setOutput();
    FastPin<ILI9341_SCLK_PIN>::setOutput();
    FastPin<ILI9341_MOSI_PIN>::lo();
    FastPin<ILI9341_SCLK_PIN>::lo();
#endif

    FastPin<ILI9341_CS_PIN>::hi(); // CS <= HIGH (deselected, so no spurious data)
    FastPin<ILI9341_DC_PIN>::hi(); // RS <= HIGH (default data byte)

#if defined(AVR_HARDWARE_SPI)
    uint8_t oldSPCR;
    uint8_t oldSPSR;
    if (ILI9341_SAVE_SPI_SETTINGS && ILI9341_RESTORE_PREV_SPI_SETTINGS) {
        oldSPCR = SPCR; // save current SPCR settings
        oldSPSR = SPSR;
    }

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!) [1 byte every 18 cycles]
#endif

#if defined(AVR_HARDWARE_SPI)
    if (ILI9341_SAVE_SPI_SETTINGS) {
        save_SPCR = SPCR; // save new initial SPCR settings
        save_SPSR = SPSR; // save new initial SPSR settings
        if (ILI9341_RESTORE_PREV_SPI_SETTINGS) {
            SPCR = oldSPCR;       // restore previous SPCR settings (spi_begin/spi_end will switch between the two)
            SPSR = oldSPSR & 0x1; // restore previous SPSR settings (spi_begin/spi_end will switch between the two)
        }
    }
#endif
    spi_begin();

    // Initialization commands for ILI9341 screens
    static const uint8_t ILI9341_cmds[] PROGMEM =
        {
            22,
            ILI9341_SWRESET, DELAY, // 1
            5,
            0xEF, 3, // 2
            0x03, 0x80, 0x02,
            0xCF, 3, // 3
            0x00, 0xC1, 0x30,
            0xED, 4, // 4
            0x64, 0x03, 0x12, 0x81,
            0xE8, 3, // 5
            0x85, 0x00, 0x78,
            0xCB, 5, // 6
            0x39, 0x2C, 0x00, 0x34, 0x02,
            0xF7, 1, // 7
            0x20,
            0xEA, 2, // 8
            0x00, 0x00,
            ILI9341_PWCTR1, 1, // 9 power control
            0x23,              // VRH[5:0]
            ILI9341_PWCTR2, 1, // 10 power control
            0x10,              // SAP[2:0];BT[3:0]
            ILI9341_VMCTR1, 2, // 11 VCM control
            0x3e, 0x28,
            ILI9341_VMCTR2, 1, // 12 VCM control2
            0x86,              // --
            ILI9341_MADCTL, 1, // 13
            (ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR),
            ILI9341_PIXFMT, 1, // 14
            0x55,
            ILI9341_FRMCTR1, 2, // 15
            0x00, 0x18,
            ILI9341_DFUNCTR, 3, // 16
            0x08, 0x82, 0x27,
            0xF2, 1, // 17 3Gamma Function Disable
            0x00,
            ILI9341_GAMMASET, 1, // 18 Gamma curve selected
            0x01,
            ILI9341_GMCTRP1, 15, // 19 Set Gamma
            0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
            ILI9341_GMCTRN1, 15, // 20
            0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
            ILI9341_SLPOUT, DELAY, // 21
            120,
            ILI9341_DISPON, 0, // 22
        };

    commandList(ILI9341_cmds);

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::setAddrWindow(coord_t x0, coord_t y0, coord_t x1, coord_t y1) {
    spi_begin();

    setAddrWindow_(x0, y0, x1, y1);

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::pushColor(color_t color) {
    spi_begin();

    spiWrite16_preCmd(color);

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::pushColor(color_t color, int count) {
    spi_begin();

    spiWrite16(color, count);

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::drawPixel(coord_t x, coord_t y, color_t color) {
    if ((x < 0) || (x >= _PARENT::_width) || (y < 0) || (y >= _PARENT::_height))
        return;

    spi_begin();

    setAddrWindow_(x, y, x, y);

    spiWrite16_preCmd(color);

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::drawFastVLine(coord_t x, s_coord_t y, coord_t h, color_t color) {
    // clipping
    if ((x < 0) || (x >= _PARENT::_width) || (y >= _PARENT::_height))
        return;

    if (y < 0) {
        h += y;
        y = 0;
    }

    s_coord_t y1 = y + h;

    if (y1 < 0)
        return;

    if (y1 > _PARENT::_height)
        h = _PARENT::_height - y;

    spi_begin();

    setAddrWindow_(x, y, x, _PARENT::_height);
    spiWrite16(color, h);

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::drawFastHLine(s_coord_t x, coord_t y, coord_t w, color_t color) {
    // clipping
    if ((x >= _PARENT::_width) || (y < 0) || (y >= _PARENT::_height))
        return;

    if (x < 0) {
        w += x;
        x = 0;
    }

    s_coord_t x1 = x + w;

    if (x1 < 0)
        return;

    if (x1 > _PARENT::_width)
        w = _PARENT::_width - w;

    spi_begin();

    setAddrWindow_(x, y, _PARENT::_width, y);
    spiWrite16(color, w);

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::fillRect(s_coord_t x, s_coord_t y, coord_t w, coord_t h, color_t color) {
    // rudimentary clipping (drawChar w/big text requires this)
    if ((x >= _PARENT::_width) || (y >= _PARENT::_height))
        return;
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if ((x + w) > _PARENT::_width)
        w = _PARENT::_width - x;
    if ((y + h) > _PARENT::_height)
        h = _PARENT::_height - y;

    spi_begin();

    setAddrWindow_(x, y, x + w - 1, _PARENT::_height);

    for (; h > 0; h--) {
        spiWrite16(color, w);
    }

    spi_end();
}

// Bresenham's algorithm - thx Wikipedia
_TEMPLATE_DEF
void _TEMPLATE_CLASS::drawLine(coord_t x0, coord_t y0, coord_t x1, coord_t y1, color_t color) {
#if 0 && defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__)
	drawLine_(x0, y0, x1, y1, color);
#else
    int8_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        swapValue(x0, y0);
        swapValue(x1, y1);
    }

    if (x0 > x1) {
        swapValue(x0, x1);
        swapValue(y0, y1);
    }

    if (x1 < 0)
        return;

    s_coord_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    s_coord_t err = dx / 2;
    int8_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    uint8_t setaddr = 1;

#if 0 && defined(__AVR_ATtiny85__) && !defined(__AVR_ATtiny45__)
	coord_t	end = steep ? _PARENT::_height-1 : _PARENT::_width-1;
	if (x1 > end)
		x1 = end;

	for (; x0 <= x1; x0++)
	{
		if ((x0 >= 0) && (y0 >= 0) && (y0 <= end))
			break;

		err -= dy;
		if (err < 0)
		{
			err += dx;
			y0 += ystep;
		}
	}

	if (x0 > x1)
		return;

	spi_begin();

	for (; x0 <= x1; x0++)
	{
		if (setaddr)
		{
			if (steep)
				setAddrWindow_(y0, x0, y0, end+1);
			else
				setAddrWindow_(x0, y0, end+1, y0);
			setaddr = 0;
		}
		spiWrite16_lineDraw(color);
		err -= dy;
		if (err < 0)
		{
			y0 += ystep;
			if ((y0 < 0) || (y0 > end))
				break;
			err += dx;
			setaddr = 1;
		}
	}
#else
    if (steep) // y increments every iteration (y0 is x-axis, and x0 is y-axis)
    {
        if (x1 >= _PARENT::_height)
            x1 = _PARENT::_height - 1;

        for (; x0 <= x1; x0++) {
            if ((x0 >= 0) && (y0 >= 0) && (y0 < _PARENT::_width))
                break;

            err -= dy;
            if (err < 0) {
                err += dx;
                y0 += ystep;
            }
        }

        if (x0 > x1)
            return;

        spi_begin();

        for (; x0 <= x1; x0++) {
            if (setaddr) {
                setAddrWindow_(y0, x0, y0, _PARENT::_height);
                setaddr = 0;
            }
            spiWrite16_lineDraw(color);
            err -= dy;
            if (err < 0) {
                y0 += ystep;
                if ((y0 < 0) || (y0 >= _PARENT::_width))
                    break;
                err += dx;
                setaddr = 1;
            }
#if defined(AVR_HARDWARE_SPI)
            else {
                __asm__ __volatile__(
                    "	rcall	delay10\n"
                    :
                    :
                    :);
            }
#endif
        }
    } else // x increments every iteration (x0 is x-axis, and y0 is y-axis)
    {
        if (x1 >= _PARENT::_width)
            x1 = _PARENT::_width - 1;

        for (; x0 <= x1; x0++) {
            if ((x0 >= 0) && (y0 >= 0) && (y0 < _PARENT::_height))
                break;

            err -= dy;
            if (err < 0) {
                err += dx;
                y0 += ystep;
            }
        }

        if (x0 > x1)
            return;

        spi_begin();

        for (; x0 <= x1; x0++) {
            if (setaddr) {
                setAddrWindow_(x0, y0, _PARENT::_width, y0);
                setaddr = 0;
            }
            spiWrite16_lineDraw(color);
            err -= dy;
            if (err < 0) {
                y0 += ystep;
                if ((y0 < 0) || (y0 >= _PARENT::_height))
                    break;
                err += dx;
                setaddr = 1;
            }
#if defined(AVR_HARDWARE_SPI)
            else {
                __asm__ __volatile__(
                    "	rcall	delay10\n"
                    :
                    :
                    :);
            }
#endif
        }
    }
#endif

    spi_end();
#endif
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::setRotation(uint8_t m) {
    _PARENT::rotation = (m & 3); // can't be higher than 3

    spi_begin();

    writeCommand(ILI9341_MADCTL);

    switch (_PARENT::rotation) {
        default:
        case 0:
            writeData(ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR);
            _PARENT::_width = ILI9341_TFTWIDTH;
            _PARENT::_height = ILI9341_TFTHEIGHT;
            break;
        case 1:
            writeData(ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
            _PARENT::_width = ILI9341_TFTHEIGHT;
            _PARENT::_height = ILI9341_TFTWIDTH;
            break;
        case 2:
            writeData(ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
            _PARENT::_width = ILI9341_TFTWIDTH;
            _PARENT::_height = ILI9341_TFTHEIGHT;
            break;
        case 3:
            writeData(ILI9341_MADCTL_MV | ILI9341_MADCTL_MY | ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR);
            _PARENT::_width = ILI9341_TFTHEIGHT;
            _PARENT::_height = ILI9341_TFTWIDTH;
            break;
    }

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::invertDisplay(boolean i) {
    spi_begin();

    writeCommand(i ? ILI9341_INVON : ILI9341_INVOFF);

    spi_end();
}

#undef INLINE
#undef INLINE_OPT
#undef _TEMPLATE_CLASS_DEF
#undef _TEMPLATE_DEF
#undef _TEMPLATE_CLASS
#undef _PARENT

#endif // !defined(_PDQ_ILI9341H_)
