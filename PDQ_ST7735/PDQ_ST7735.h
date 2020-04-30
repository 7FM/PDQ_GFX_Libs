// This is the PDQ re-mixed version of Adafruit's library
// here is the original copyright notice:

/***************************************************
  This is a library for the Adafruit 1.8" SPI display.

This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
The 1.8" TFT shield
  ----> https://www.adafruit.com/product/802
The 1.44" TFT breakout
  ----> https://www.adafruit.com/product/2088
as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

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
// 2) Between ~3.5 and ~12 times faster (fillRect ~2.5x, drawLine > ~14x).
// An average of almost 5x faster over entire "graphictest.ino" benchmark.
//
// Even if this library is faster, it was based on the Adafruit original.
// Adafruit deserves your support for making their library open-source (and
// for having some nice LCD modules and all kinds of other great parts too).
// Consider giving them your support if possible!

#if !defined(_PDQ_ST7735H_)
#define _PDQ_ST7735H_

#include "Arduino.h"
#include "Print.h"

#include <PDQ_GFX/PDQ_GFX.h>

#include <SPI.h>
#include <avr/pgmspace.h>

#include <FastPin.h>

#if !defined(AVR_HARDWARE_SPI)
#error Oops!  Currently hardware SPI is required.  Bother Xark if you would like USI or bit-bang SPI supported.
#endif

#define INLINE inline
#define INLINE_OPT __attribute__((always_inline))

typedef enum {
    ST7735_INITB = 0,                                // 1.8" (128x160) ST7735B chipset (only one type)
    ST7735_INITR_GREENTAB = 1,                       // 1.8" (128x160) ST7735R chipset with green tab (same as ST7735_INITR_18GREENTAB)
    ST7735_INITR_REDTAB = 2,                         // 1.8" (128x160) ST7735R chipset with red tab (same as ST7735_INITR_18REDTAB)
    ST7735_INITR_BLACKTAB = 3,                       // 1.8" (128x160) ST7735S chipset with black tab (same as ST7735_INITR_18BLACKTAB)
    ST7735_INITR_144GREENTAB = 4,                    // 1.4" (128x128) ST7735R chipset with green tab
    ST7735_INITR_18GREENTAB = ST7735_INITR_GREENTAB, // 1.8" (128x160) ST7735R chipset with green tab
    ST7735_INITR_18REDTAB = ST7735_INITR_REDTAB,     // 1.8" (128x160) ST7735R chipset with red tab
    ST7735_INITR_18BLACKTAB = ST7735_INITR_BLACKTAB, // 1.8" (128x160) ST7735S chipset with black tab
} ST7735_Chipset;

//#define ST7735_CHIPSET ST7735_INITR_REDTAB // <= Set ST7735 LCD chipset/variation here (from above list)
// NOTE: These are typical hookups individual boards will vary, please check your documentation.
// CAUTION: While Adafruit boards generally always come with needed level-converters, I find many
//          other LCD displays advertised as supporting 5V only support 5V power (with a regulator).
//          They still only have 3.3V safe logic (CS, DC, RESET, MOSI, SCK marked with * below).
//          If this is the case you will need a voltage level-converter (e.g., HC4050, divider circuit etc.).

// Color definitions
typedef enum {
    ST7735_BLACK = 0x0000,
    ST7735_BLUE = 0x001F,
    ST7735_RED = 0xF800,
    ST7735_GREEN = 0x07E0,
    ST7735_CYAN = 0x07FF,
    ST7735_MAGENTA = 0xF81F,
    ST7735_YELLOW = 0xFFE0,
    ST7735_WHITE = 0xFFFF,
} ST7735_Color;

// ST7735 commands (read commands omitted)
// For datasheet see https://www.adafruit.com/products/358
typedef enum {
    ST7735_NOP = 0x00,
    ST7735_SWRESET = 0x01,

    ST7735_SLPIN = 0x10,
    ST7735_SLPOUT = 0x11,
    ST7735_PTLON = 0x12,
    ST7735_NORON = 0x13,

    ST7735_INVOFF = 0x20,
    ST7735_INVON = 0x21,
    ST7735_DISPOFF = 0x28,
    ST7735_DISPON = 0x29,
    ST7735_CASET = 0x2A,
    ST7735_RASET = 0x2B,
    ST7735_RAMWR = 0x2C,

    ST7735_COLMOD = 0x3A,
    ST7735_MADCTL = 0x36,

    ST7735_FRMCTR1 = 0xB1,
    ST7735_FRMCTR2 = 0xB2,
    ST7735_FRMCTR3 = 0xB3,
    ST7735_INVCTR = 0xB4,
    ST7735_DISSET5 = 0xB6,

    ST7735_PWCTR1 = 0xC0,
    ST7735_PWCTR2 = 0xC1,
    ST7735_PWCTR3 = 0xC2,
    ST7735_PWCTR4 = 0xC3,
    ST7735_PWCTR5 = 0xC4,
    ST7735_VMCTR1 = 0xC5,

    ST7735_GMCTRP1 = 0xE0,
    ST7735_GMCTRN1 = 0xE1,

    ST7735_PWCTR6 = 0xFC,
} ST7735_Command;

// some other misc. constants
enum {
    // screen dimensions
    ST7735_TFTWIDTH = 128,
    ST7735_TFTHEIGHT_144 = 128, // 1.44" display
    ST7735_TFTHEIGHT_18 = 160,  // 1.8" display

    // MADCTL bits
    ST7735_MADCTL_MH = 0x04,  // bit 2 = 0 for refresh left -> right, 1 for refresh right -> left
    ST7735_MADCTL_RGB = 0x00, // bit 3 = 0 for RGB color order
    ST7735_MADCTL_BGR = 0x08, // bit 3 = 1 for BGR color order
    ST7735_MADCTL_ML = 0x10,  // bit 4 = 0 for refresh top -> bottom, 1 for bottom -> top
    ST7735_MADCTL_MV = 0x20,  // bit 5 = 0 for column, row order (portrait), 1 for row, column order (landscape)
    ST7735_MADCTL_MX = 0x40,  // bit 6 = 0 for left -> right, 1 for right -> left order
    ST7735_MADCTL_MY = 0x80,  // bit 7 = 0 for top -> bottom, 1 for bottom -> top

    // delay indicator bit for commandList()
    DELAY = 0x80
};

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

// 13 cycle delay (including "rcall")
__attribute__((noinline)) __attribute__((naked)) __attribute__((used)) void delay13() {
    __asm__ __volatile__(
        // +3 (rcall to get here)
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
        "	nop\n"           // +1 (1-cycle NOP)
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
        "	rcall	delay17\n" // rcall unmangled delay17 (compiler would needlessly save/restore regs)
        :
        :
        :);
}

// special SPI write with minimal hand-tuned delay (assuming max DIV2 SPI rate) - minus 2 cycles for RS (etc.) change
static INLINE INLINE_OPT void spiWrite_preCmd(uint8_t data) {
    SPDR = data;

    __asm__ __volatile__(
        "	rcall	delay15\n" // rcall unmangled delay15 (compiler would needlessly save/restore regs)
        :
        :
        :);
}

// SPI 16-bit write with minimal hand-tuned delay (assuming max DIV2 SPI rate)
static INLINE INLINE_OPT void spiWrite16(uint16_t data) {
    uint8_t temp;
    __asm__ __volatile__(
        "	out	%[spi],%[hi]\n" // write SPI data (18 cycles until next write)
        "	rcall	delay17\n"  // rcall unmangled delay17 (compiler would needlessly save/restore regs)
        "	out	%[spi],%[lo]\n" // write SPI data (18 cycles until next write)
        "	rcall	delay17\n"  // rcall unmangled delay17 (compiler would needlessly save/restore regs)

        : [ temp ] "=d"(temp)
        : [ spi ] "i"(_SFR_IO_ADDR(SPDR)), [ lo ] "r"((uint8_t)data), [ hi ] "r"((uint8_t)(data >> 8))
        :);
}

// SPI 16-bit write with minimal hand-tuned delay (assuming max DIV2 SPI rate) minus 2 cycles
static INLINE INLINE_OPT void spiWrite16_preCmd(uint16_t data) {
    uint8_t temp;
    __asm__ __volatile__(
        "	out	%[spi],%[hi]\n" // write SPI data (18 cycles until next write)
        "	rcall	delay17\n"  // rcall unmangled delay17 (compiler would needlessly save/restore regs)
        "	out	%[spi],%[lo]\n" // write SPI data (18 cycles until next write)
        "	rcall	delay15\n"  // rcall unmangled delay15 (compiler would needlessly save/restore regs)

        : [ temp ] "=d"(temp)
        : [ spi ] "i"(_SFR_IO_ADDR(SPDR)), [ lo ] "r"((uint8_t)data), [ hi ] "r"((uint8_t)(data >> 8))
        :);
}

// SPI 16-bit write with minimal hand-tuned delay (assuming max DIV2 SPI rate) for drawLine
static INLINE INLINE_OPT void spiWrite16_lineDraw(uint16_t data) {
    uint8_t temp;
    __asm__ __volatile__(
        "	out	%[spi],%[hi]\n" // write SPI data (18 cycles until next write)
        "	rcall	delay17\n"  // rcall unmangled delay17 (compiler would needlessly save/restore regs)
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
        "	rcall	delay17\n"    // rcall unmangled delay17 (compiler would needlessly save/restore regs)
        "	out	%[spi],%[lo]\n"   // write SPI data (18 cycles until next write)
        "	rcall	delay13\n"    // rcall unmangled delay13 (compiler would needlessly save/restore regs)
        "	sbiw	%[count],1\n" // +2	decrement count
        "	brne	1b\n"         // +2/1	if != 0 then loop
                                  // = 13 + 2 + 2 (17 cycles)
        "4:\n"

        : [ temp ] "=d"(temp), [ count ] "+w"(count)
        : [ spi ] "i"(_SFR_IO_ADDR(SPDR)), [ lo ] "r"((uint8_t)data), [ hi ] "r"((uint8_t)(data >> 8))
        :);
}

// Ugly... I know but it works and you cant have more than one display either with this library
// May the compiler remove optimize it away if unused x.x
static volatile uint8_t save_SPCR; // initial SPCR value/saved SPCR value (swapped in spi_begin/spi_end)

#define _TEMPLATE_DEF template <ST7735_Chipset ST7735_CHIPSET, uint8_t ST7735_CS_PIN, uint8_t ST7735_DC_PIN, uint8_t ST7735_RST_PIN, bool ST7735_SAVE_SPCR>

#define _TEMPLATE_CLASS PDQ_ST7735<ST7735_CHIPSET, ST7735_CS_PIN, ST7735_DC_PIN, ST7735_RST_PIN, ST7735_SAVE_SPCR>
#define _PARENT PDQ_GFX<_TEMPLATE_CLASS, ST7735_TFTWIDTH, ST7735_TFTHEIGHT_18>
_TEMPLATE_DEF
class PDQ_ST7735 : public _PARENT {
  public:
    // higher-level routines
    static void inline begin();

    static void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    static void pushColor(uint16_t color);
    static void pushColor(uint16_t color, int count);

    // required driver primitive methods (all except drawPixel can call generic version in PDQ_GFX with "_" postfix).
    static void drawPixel(int x, int y, uint16_t color);
    static void drawFastVLine(int x, int y, int h, uint16_t color);
    static void drawFastHLine(int x, int y, int w, uint16_t color);
    static void setRotation(uint8_t r);
    static void invertDisplay(boolean i);

    static inline void fillScreen(uint16_t color) __attribute__((always_inline)) {
        _PARENT::fillScreen_(color); // call generic version
    }

    static void drawLine(int x0, int y0, int x1, int y1, uint16_t color);
    static void fillRect(int x, int y, int w, int h, uint16_t color);

    // Pass 8-bit (each) R,G,B, get back 16-bit packed color
    static constexpr inline uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }

    // === lower-level internal routines =========
    static void commandList(const uint8_t *addr);

  private:
    // NOTE: Make sure each spi_begin() is matched with a single spi_end() (and don't call either twice)
    // set CS back to low (LCD selected)
    static inline void spi_begin() __attribute__((always_inline)) {

        if (ST7735_SAVE_SPCR) {
            swapValue(save_SPCR, SPCR); // swap initial/current SPCR settings
        }
        FastPin<ST7735_CS_PIN>::lo(); // CS <= LOW (selected)
    }

    // NOTE: Make sure each spi_begin() is matched with a single spi_end() (and don't call either twice)
    // reset CS back to high (LCD unselected)
    static inline void spi_end() __attribute__((always_inline)) {
        FastPin<ST7735_CS_PIN>::hi(); // CS <= HIGH (deselected)
        if (ST7735_SAVE_SPCR) {
            swapValue(SPCR, save_SPCR); // swap current/initial SPCR settings
        }
    }

    // write SPI byte with RS (aka D/C) pin set low to indicate a command byte (and then reset back to high when done)
    static INLINE void writeCommand(uint8_t data) INLINE_OPT {
        FastPin<ST7735_DC_PIN>::lo(); // RS <= LOW indicate command byte
        spiWrite_preCmd(data);
        FastPin<ST7735_DC_PIN>::hi(); // RS <= HIGH indicate data byte (always assumed left in data mode)
    }

    // write SPI byte with RS assumed low indicating a data byte
    static inline void writeData(uint8_t data) __attribute__((always_inline)) {
        spiWrite(data);
    }

    // internal version that does not spi_begin()/spi_end()
    static INLINE void setAddrWindow_(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) INLINE_OPT {
        writeCommand(ST7735_CASET); // column address set
        if ((ST7735_CHIPSET == ST7735_INITR_GREENTAB) || (ST7735_CHIPSET == ST7735_INITR_144GREENTAB)) {
            spiWrite16(x0 + 2);        // XSTART
            spiWrite16_preCmd(x1 + 2); // XEND
        } else {
            spiWrite16(x0);        // XSTART
            spiWrite16_preCmd(x1); // XEND
        }

        writeCommand(ST7735_RASET); // row address set
        if (ST7735_CHIPSET == ST7735_INITR_GREENTAB) {
            spiWrite16(y0 + 1);        // YSTART
            spiWrite16_preCmd(y1 + 1); // YEND
        } else if (ST7735_CHIPSET == ST7735_INITR_144GREENTAB) {
            spiWrite16(y0 + 3);        // YSTART
            spiWrite16_preCmd(y1 + 3); // YEND
        } else {
            spiWrite16(y0);        // YSTART
            spiWrite16_preCmd(y1); // YEND
        }

        writeCommand(ST7735_RAMWR); // write to RAM
    }
};

_TEMPLATE_DEF
void _TEMPLATE_CLASS::begin() {

    FastPin<ST7735_RST_PIN>::setOutput();
    FastPin<ST7735_RST_PIN>::lo();

#ifdef __AVR
    _delay_ms(1);
#else
    delay(1);
#endif
    FastPin<ST7735_RST_PIN>::hi();

    // set CS and RS pin directions to output
    FastPin<ST7735_CS_PIN>::setOutput();
    FastPin<ST7735_DC_PIN>::setOutput();

    FastPin<ST7735_CS_PIN>::hi(); // CS <= HIGH (so no spurious data)
    FastPin<ST7735_DC_PIN>::hi(); // RS <= HIGH (default data byte)

    uint8_t oldSPCR;
    if (ST7735_SAVE_SPCR) {
        oldSPCR = SPCR; // save current SPCR settings
    }

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!) [1 byte every 18 cycles]

    if (ST7735_SAVE_SPCR) {
        save_SPCR = SPCR; // save new initial SPCR settings
        SPCR = oldSPCR;   // restore previous SPCR settings (spi_begin/spi_end will switch between the two)
    }
    spi_begin();

    // ST7735B
    if (ST7735_CHIPSET == ST7735_INITB) {
        // Initialization commands for ST7735B screens
        static const uint8_t PROGMEM Bcmd[] = // ==== Initialization commands for 7735B screens
            {
                18,                        // 18 commands in list:
                ST7735_SWRESET, DELAY,     //  1: Software reset, no args, w/delay
                50,                        //     50 ms delay
                ST7735_SLPOUT, DELAY,      //  2: Out of sleep mode, no args, w/delay
                255,                       //     255 = 500 ms delay
                ST7735_COLMOD, 1 + DELAY,  //  3: Set color mode, 1 arg + delay:
                0x05,                      //     16-bit color
                10,                        //     10 ms delay
                ST7735_FRMCTR1, 3 + DELAY, //  4: Frame rate control, 3 args + delay:
                0x00,                      //     fastest refresh
                0x06,                      //     6 lines front porch
                0x03,                      //     3 lines back porch
                10,                        //     10 ms delay
                ST7735_MADCTL, 1,          //  5: Memory access ctrl (directions), 1 arg:
                0x08,                      //     Row addr/col addr, bottom to top refresh
                ST7735_DISSET5, 2,         //  6: Display settings #5, 2 args, no delay:
                0x15,                      //     1 clk cycle nonoverlap, 2 cycle gate
                                           //     rise, 3 cycle osc equalize
                0x02,                      //     Fix on VTL
                ST7735_INVCTR, 1,          //  7: Display inversion control, 1 arg:
                0x0,                       //     Line inversion
                ST7735_PWCTR1, 2 + DELAY,  //  8: Power control, 2 args + delay:
                0x02,                      //     GVDD = 4.7V
                0x70,                      //     1.0uA
                10,                        //     10 ms delay
                ST7735_PWCTR2, 1,          //  9: Power control, 1 arg, no delay:
                0x05,                      //     VGH = 14.7V, VGL = -7.35V
                ST7735_PWCTR3, 2,          // 10: Power control, 2 args, no delay:
                0x01,                      //     Opamp current small
                0x02,                      //     Boost frequency
                ST7735_VMCTR1, 2 + DELAY,  // 11: Power control, 2 args + delay:
                0x3C,                      //     VCOMH = 4V
                0x38,                      //     VCOML = -1.1V
                10,                        //     10 ms delay
                ST7735_PWCTR6, 2,          // 12: Power control, 2 args, no delay:
                0x11, 0x15,
                ST7735_GMCTRP1, 16,     // 13: Magical unicorn dust, 16 args, no delay:
                0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
                0x21, 0x1B, 0x13, 0x19, //      these config values represent)
                0x17, 0x15, 0x1E, 0x2B,
                0x04, 0x05, 0x02, 0x0E,
                ST7735_GMCTRN1, 16 + DELAY, // 14: Sparkles and rainbows, 16 args + delay:
                0x0B, 0x14, 0x08, 0x1E,     //     (ditto)
                0x22, 0x1D, 0x18, 0x1E,
                0x1B, 0x1A, 0x24, 0x2B,
                0x06, 0x06, 0x02, 0x0F,
                10,                   //     10 ms delay
                ST7735_CASET, 4,      // 15: Column addr set, 4 args, no delay:
                0x00, 0x02,           //     XSTART = 2
                0x00, 0x81,           //     XEND = 129
                ST7735_RASET, 4,      // 16: Row addr set, 4 args, no delay:
                0x00, 0x02,           //     XSTART = 1
                0x00, 0x81,           //     XEND = 160
                ST7735_NORON, DELAY,  // 17: Normal display on, no args, w/delay
                10,                   //     10 ms delay
                ST7735_DISPON, DELAY, // 18: Main screen turn on, no args, w/delay
                255                   //     255 = 500 ms delay
            };
        commandList(Bcmd);
    } else {
        static const uint8_t PROGMEM Rcmd1[] = // ==== Init for 7735R, part 1 (red or green tab)
            {
                15,                    // 15 commands in list:
                ST7735_SWRESET, DELAY, //  1: Software reset, 0 args, w/delay
                150,                   //     150 ms delay
                ST7735_SLPOUT, DELAY,  //  2: Out of sleep mode, 0 args, w/delay
                255,                   //     500 ms delay
                ST7735_FRMCTR1, 3,     //  3: Frame rate ctrl - normal mode, 3 args:
                0x01, 0x2C, 0x2D,      //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
                ST7735_FRMCTR2, 3,     //  4: Frame rate control - idle mode, 3 args:
                0x01, 0x2C, 0x2D,      //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
                ST7735_FRMCTR3, 6,     //  5: Frame rate ctrl - partial mode, 6 args:
                0x01, 0x2C, 0x2D,      //     Dot inversion mode
                0x01, 0x2C, 0x2D,      //     Line inversion mode
                ST7735_INVCTR, 1,      //  6: Display inversion ctrl, 1 arg, no delay:
                0x07,                  //     No inversion
                ST7735_PWCTR1, 3,      //  7: Power control, 3 args, no delay:
                0xA2,
                0x02,             //     -4.6V
                0x84,             //     AUTO mode
                ST7735_PWCTR2, 1, //  8: Power control, 1 arg, no delay:
                0xC5,             //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
                ST7735_PWCTR3, 2, //  9: Power control, 2 args, no delay:
                0x0A,             //     Opamp current small
                0x00,             //     Boost frequency
                ST7735_PWCTR4, 2, // 10: Power control, 2 args, no delay:
                0x8A,             //     BCLK/2, Opamp current small & Medium low
                0x2A,
                ST7735_PWCTR5, 2, // 11: Power control, 2 args, no delay:
                0x8A, 0xEE,
                ST7735_VMCTR1, 1, // 12: Power control, 1 arg, no delay:
                0x0E,
                ST7735_INVOFF, 0, // 13: Don't invert display, no args, no delay
                ST7735_MADCTL, 1, // 14: Memory access control (directions), 1 arg:
                0xC8,             //     row addr/col addr, bottom to top refresh
                ST7735_COLMOD, 1, // 15: set color mode, 1 arg, no delay:
                0x05              //     16-bit color
            };
        // ST7735R with a few types denoted by "tab type" (color of tab on LCD cover sheet) and 144 for 1.44" LCD
        commandList(Rcmd1);
        if (ST7735_CHIPSET == ST7735_INITR_GREENTAB) {
            static const uint8_t PROGMEM Rcmd2green[] = // ==== Init for 7735R, part 2 (green tab only)
                {
                    2,                 //  2 commands in list:
                    ST7735_CASET, 4,   //  1: Column addr set, 4 args, no delay:
                    0x00, 0x02,        //     XSTART = 0
                    0x00, 0x7F + 0x02, //     XEND = 127
                    ST7735_RASET, 4,   //  2: Row addr set, 4 args, no delay:
                    0x00, 0x01,        //     XSTART = 0
                    0x00, 0x9F + 0x01  //     XEND = 159
                };
            commandList(Rcmd2green);
        } else if (ST7735_CHIPSET == ST7735_INITR_144GREENTAB) {
            static const uint8_t PROGMEM Rcmd2green144[] = // ==== Init for 7735R, part 2 (green 1.44 tab)
                {
                    2,               //  2 commands in list:
                    ST7735_CASET, 4, //  1: Column addr set, 4 args, no delay:
                    0x00, 0x00,      //     XSTART = 0
                    0x00, 0x7F,      //     XEND = 127
                    ST7735_RASET, 4, //  2: Row addr set, 4 args, no delay:
                    0x00, 0x00,      //     XSTART = 0
                    0x00, 0x7F       //     XEND = 127
                };
            _PARENT::_height = ST7735_TFTHEIGHT_144;
            commandList(Rcmd2green144);
        } else {
            static const uint8_t PROGMEM Rcmd2red[] = // Init for 7735R, part 2 (red tab only)
                {
                    2,               //  2 commands in list:
                    ST7735_CASET, 4, //  1: Column addr set, 4 args, no delay:
                    0x00, 0x00,      //     XSTART = 0
                    0x00, 0x7F,      //     XEND = 127
                    ST7735_RASET, 4, //  2: Row addr set, 4 args, no delay:
                    0x00, 0x00,      //     XSTART = 0
                    0x00, 0x9F       //     XEND = 159
                };
            commandList(Rcmd2red);
        }

        static const uint8_t PROGMEM Rcmd3[] = // ==== Init for 7735R, part 3 (red or green tab)
        {
            4, //  4 commands in list:
            ST7735_GMCTRP1,
            16, //  1: Magical unicorn dust, 16 args, no delay:
#if 0           // Adafruit or alternate gamma settings...
		      0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
#else
            0x0f,
            0x1a,
            0x0f,
            0x18,
            0x2f,
            0x28,
            0x20,
            0x22,
            0x1f,
            0x1b,
            0x23,
            0x37,
            0x00,
            0x07,
            0x02,
            0x10,
#endif
            ST7735_GMCTRN1,
            16, //  2: Sparkles and rainbows, 16 args, no delay:
#if 0           // Adafruit or alternate gamma settings...
		      0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
#else
            0x0f,
            0x1b,
            0x0f,
            0x17,
            0x33,
            0x2c,
            0x29,
            0x2e,
            0x30,
            0x30,
            0x39,
            0x3f,
            0x00,
            0x07,
            0x03,
            0x10,
#endif
            ST7735_NORON,
            DELAY, //  3: Normal display on, no args, w/delay
            10,    //     10 ms delay
            ST7735_DISPON,
            DELAY, //  4: Main screen turn on, no args w/delay
            100    //     100 ms delay
        };

        commandList(Rcmd3);
        if (ST7735_CHIPSET == ST7735_INITR_BLACKTAB) {
            // ST7735S has a few differences
            writeCommand(ST7735_MADCTL);
            writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB);
        }
    }

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    spi_begin();

    setAddrWindow_(x0, y0, x1, y1);

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::pushColor(uint16_t color) {
    spi_begin();

    spiWrite16_preCmd(color);

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::pushColor(uint16_t color, int count) {
    spi_begin();

    spiWrite16(color, count);

    spi_end();
}

// required driver primitive methods (all except drawPixel can call generic version in PDQ_GFX with "_" postfix).
_TEMPLATE_DEF
void _TEMPLATE_CLASS::drawPixel(int x, int y, uint16_t color) {
    if ((x < 0) || (x >= _PARENT::_width) || (y < 0) || (y >= _PARENT::_height))
        return;

    spi_begin();

    setAddrWindow_(x, y, x, y);

    spiWrite16_preCmd(color);

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::drawFastVLine(int x, int y, int h, uint16_t color) {
    // clipping
    if ((x < 0) || (x >= _PARENT::_width) || (y >= _PARENT::_height))
        return;

    if (y < 0) {
        h += y;
        y = 0;
    }

    int y1 = y + h;

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
void _TEMPLATE_CLASS::drawFastHLine(int x, int y, int w, uint16_t color) {
    // clipping
    if ((x >= _PARENT::_width) || (y < 0) || (y >= _PARENT::_height))
        return;

    if (x < 0) {
        w += x;
        x = 0;
    }

    int x1 = x + w;

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
void _TEMPLATE_CLASS::setRotation(uint8_t r) {
    _PARENT::rotation = (r & 3); // can't be higher than 3

    spi_begin();

    writeCommand(ST7735_MADCTL);

    switch (_PARENT::rotation) {
        default:
        case 0:
            if (ST7735_CHIPSET == ST7735_INITR_BLACKTAB)
                writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB);
            else
                writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR);
            _PARENT::_width = ST7735_TFTWIDTH;
            if (ST7735_CHIPSET == ST7735_INITR_144GREENTAB)
                _PARENT::_height = ST7735_TFTHEIGHT_144;
            else
                _PARENT::_height = ST7735_TFTHEIGHT_18;

            break;
        case 1:
            if (ST7735_CHIPSET == ST7735_INITR_BLACKTAB)
                writeData(ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_RGB);
            else
                writeData(ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR);
            if (ST7735_CHIPSET == ST7735_INITR_144GREENTAB)
                _PARENT::_width = ST7735_TFTHEIGHT_144;
            else
                _PARENT::_width = ST7735_TFTHEIGHT_18;
            _PARENT::_height = ST7735_TFTWIDTH;
            break;
        case 2:
            if (ST7735_CHIPSET == ST7735_INITR_BLACKTAB)
                writeData(ST7735_MADCTL_RGB);
            else
                writeData(ST7735_MADCTL_BGR);
            _PARENT::_width = ST7735_TFTWIDTH;
            if (ST7735_CHIPSET == ST7735_INITR_144GREENTAB)
                _PARENT::_height = ST7735_TFTHEIGHT_144;
            else
                _PARENT::_height = ST7735_TFTHEIGHT_18;
            break;
        case 3:
            if (ST7735_CHIPSET == ST7735_INITR_BLACKTAB)
                writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_RGB);
            else
                writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR);
            if (ST7735_CHIPSET == ST7735_INITR_144GREENTAB)
                _PARENT::_width = ST7735_TFTHEIGHT_144;
            else
                _PARENT::_width = ST7735_TFTHEIGHT_18;
            _PARENT::_height = ST7735_TFTWIDTH;
            break;
    }

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::invertDisplay(boolean i) {
    spi_begin();

    writeCommand(i ? ST7735_INVON : ST7735_INVOFF);

    spi_end();
}

// Bresenham's algorithm - thx Wikipedia
_TEMPLATE_DEF
void _TEMPLATE_CLASS::drawLine(int x0, int y0, int x1, int y1, uint16_t color) {
    int8_t steep = abs(y1 - y0) > abs(x1 - x0);
    coord_t xBorder;
    coord_t yBorder;
    if (steep) {
        // y increments every iteration (y0 is x-axis, and x0 is y-axis)
        xBorder = _PARENT::_height;
        yBorder = _PARENT::_width;
        swapValue(x0, y0);
        swapValue(x1, y1);
    } else {
        // x increments every iteration (x0 is x-axis, and y0 is y-axis)
        xBorder = _PARENT::_width;
        yBorder = _PARENT::_height;
    }

    if (x0 > x1) {
        swapValue(x0, x1);
        swapValue(y0, y1);
    }

    if (x1 < 0)
        return;

    int dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int err = dx / 2;
    int8_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    uint8_t setaddr = 1;

    if (x1 >= xBorder)
        x1 = xBorder - 1;

    for (; x0 <= x1; ++x0) {
        if ((x0 >= 0) && (y0 >= 0) && (y0 < yBorder))
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

    for (; x0 <= x1; ++x0) {
        if (setaddr) {
            // We need consider steep orientation here as well
            if (steep) {
                setAddrWindow_(x0, y0, y0, xBorder);
            } else {
                setAddrWindow_(x0, y0, xBorder, y0);
            }
            setaddr = 0;
        }
        spiWrite16_lineDraw(color);
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            if ((y0 < 0) || (y0 >= yBorder))
                break;
            err += dx;
            setaddr = 1;
        } else {
            __asm__ __volatile__(
                "	rcall	delay10\n"
                :
                :
                :);
        }
    }

    spi_end();
}

_TEMPLATE_DEF
void _TEMPLATE_CLASS::fillRect(int x, int y, int w, int h, uint16_t color) {
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

// === lower-level internal routines =========
// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
_TEMPLATE_DEF
void _TEMPLATE_CLASS::commandList(const uint8_t *addr) {
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = pgm_read_byte(addr++); // Number of commands to follow
                                         // For each command...
    while (numCommands--) {
        writeCommand(pgm_read_byte(addr++)); // Read, issue command
        numArgs = pgm_read_byte(addr++);     // Number of args to follow
        ms = numArgs & DELAY;                // If hibit set, delay follows args
        numArgs &= ~DELAY;                   // Mask out delay bit
        // For each argument...
        while (numArgs--) {
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

#endif // !defined(_PDQ_ST7735H_)
