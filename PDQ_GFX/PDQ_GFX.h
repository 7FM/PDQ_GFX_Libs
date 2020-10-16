// This is the PDQ re-mixed version of Adafruit's library from Xark
// here is the original copyright notice and license:

/*
This is the core graphics library for all our displays, providing a common
set of graphics primitives (points, lines, circles, etc.).	It needs to be
paired with a hardware-specific library for each display device we carry
(to handle the lower-level functions).

Adafruit invests time and resources providing this open source code, please
support Adafruit & open-source hardware by purchasing products from Adafruit!

Copyright (c) 2013 Adafruit Industries.	All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

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

#ifndef _PDQ_GFX_H
#define _PDQ_GFX_H

#include "Arduino.h"
#include "Print.h"

#define X_PIXEL_PER_CHAR 6
#define Y_PIXEL_PER_CHAR 8

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
#ifndef pgm_read_word
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif
#ifndef pgm_read_dword
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#endif
#if !defined(__INT_MAX__) || (__INT_MAX__ > 0xFFFFL)
#define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr))
#else
#define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))
#endif

// Standard ASCII 5x7 font

const unsigned char glcdfont[] PROGMEM =
    {
        0x00, 0x00, 0x00, 0x00, 0x00,
        0x3E, 0x5B, 0x4F, 0x5B, 0x3E,
        0x3E, 0x6B, 0x4F, 0x6B, 0x3E,
        0x1C, 0x3E, 0x7C, 0x3E, 0x1C,
        0x18, 0x3C, 0x7E, 0x3C, 0x18,
        0x1C, 0x57, 0x7D, 0x57, 0x1C,
        0x1C, 0x5E, 0x7F, 0x5E, 0x1C,
        0x00, 0x18, 0x3C, 0x18, 0x00,
        0xFF, 0xE7, 0xC3, 0xE7, 0xFF,
        0x00, 0x18, 0x24, 0x18, 0x00,
        0xFF, 0xE7, 0xDB, 0xE7, 0xFF,
        0x30, 0x48, 0x3A, 0x06, 0x0E,
        0x26, 0x29, 0x79, 0x29, 0x26,
        0x40, 0x7F, 0x05, 0x05, 0x07,
        0x40, 0x7F, 0x05, 0x25, 0x3F,
        0x5A, 0x3C, 0xE7, 0x3C, 0x5A,
        0x7F, 0x3E, 0x1C, 0x1C, 0x08,
        0x08, 0x1C, 0x1C, 0x3E, 0x7F,
        0x14, 0x22, 0x7F, 0x22, 0x14,
        0x5F, 0x5F, 0x00, 0x5F, 0x5F,
        0x06, 0x09, 0x7F, 0x01, 0x7F,
        0x00, 0x66, 0x89, 0x95, 0x6A,
        0x60, 0x60, 0x60, 0x60, 0x60,
        0x94, 0xA2, 0xFF, 0xA2, 0x94,
        0x08, 0x04, 0x7E, 0x04, 0x08,
        0x10, 0x20, 0x7E, 0x20, 0x10,
        0x08, 0x08, 0x2A, 0x1C, 0x08,
        0x08, 0x1C, 0x2A, 0x08, 0x08,
        0x1E, 0x10, 0x10, 0x10, 0x10,
        0x0C, 0x1E, 0x0C, 0x1E, 0x0C,
        0x30, 0x38, 0x3E, 0x38, 0x30,
        0x06, 0x0E, 0x3E, 0x0E, 0x06,
        0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x5F, 0x00, 0x00,
        0x00, 0x07, 0x00, 0x07, 0x00,
        0x14, 0x7F, 0x14, 0x7F, 0x14,
        0x24, 0x2A, 0x7F, 0x2A, 0x12,
        0x23, 0x13, 0x08, 0x64, 0x62,
        0x36, 0x49, 0x56, 0x20, 0x50,
        0x00, 0x08, 0x07, 0x03, 0x00,
        0x00, 0x1C, 0x22, 0x41, 0x00,
        0x00, 0x41, 0x22, 0x1C, 0x00,
        0x2A, 0x1C, 0x7F, 0x1C, 0x2A,
        0x08, 0x08, 0x3E, 0x08, 0x08,
        0x00, 0x80, 0x70, 0x30, 0x00,
        0x08, 0x08, 0x08, 0x08, 0x08,
        0x00, 0x00, 0x60, 0x60, 0x00,
        0x20, 0x10, 0x08, 0x04, 0x02,
        0x3E, 0x51, 0x49, 0x45, 0x3E,
        0x00, 0x42, 0x7F, 0x40, 0x00,
        0x72, 0x49, 0x49, 0x49, 0x46,
        0x21, 0x41, 0x49, 0x4D, 0x33,
        0x18, 0x14, 0x12, 0x7F, 0x10,
        0x27, 0x45, 0x45, 0x45, 0x39,
        0x3C, 0x4A, 0x49, 0x49, 0x31,
        0x41, 0x21, 0x11, 0x09, 0x07,
        0x36, 0x49, 0x49, 0x49, 0x36,
        0x46, 0x49, 0x49, 0x29, 0x1E,
        0x00, 0x00, 0x14, 0x00, 0x00,
        0x00, 0x40, 0x34, 0x00, 0x00,
        0x00, 0x08, 0x14, 0x22, 0x41,
        0x14, 0x14, 0x14, 0x14, 0x14,
        0x00, 0x41, 0x22, 0x14, 0x08,
        0x02, 0x01, 0x59, 0x09, 0x06,
        0x3E, 0x41, 0x5D, 0x59, 0x4E,
        0x7C, 0x12, 0x11, 0x12, 0x7C,
        0x7F, 0x49, 0x49, 0x49, 0x36,
        0x3E, 0x41, 0x41, 0x41, 0x22,
        0x7F, 0x41, 0x41, 0x41, 0x3E,
        0x7F, 0x49, 0x49, 0x49, 0x41,
        0x7F, 0x09, 0x09, 0x09, 0x01,
        0x3E, 0x41, 0x41, 0x51, 0x73,
        0x7F, 0x08, 0x08, 0x08, 0x7F,
        0x00, 0x41, 0x7F, 0x41, 0x00,
        0x20, 0x40, 0x41, 0x3F, 0x01,
        0x7F, 0x08, 0x14, 0x22, 0x41,
        0x7F, 0x40, 0x40, 0x40, 0x40,
        0x7F, 0x02, 0x1C, 0x02, 0x7F,
        0x7F, 0x04, 0x08, 0x10, 0x7F,
        0x3E, 0x41, 0x41, 0x41, 0x3E,
        0x7F, 0x09, 0x09, 0x09, 0x06,
        0x3E, 0x41, 0x51, 0x21, 0x5E,
        0x7F, 0x09, 0x19, 0x29, 0x46,
        0x26, 0x49, 0x49, 0x49, 0x32,
        0x03, 0x01, 0x7F, 0x01, 0x03,
        0x3F, 0x40, 0x40, 0x40, 0x3F,
        0x1F, 0x20, 0x40, 0x20, 0x1F,
        0x3F, 0x40, 0x38, 0x40, 0x3F,
        0x63, 0x14, 0x08, 0x14, 0x63,
        0x03, 0x04, 0x78, 0x04, 0x03,
        0x61, 0x59, 0x49, 0x4D, 0x43,
        0x00, 0x7F, 0x41, 0x41, 0x41,
        0x02, 0x04, 0x08, 0x10, 0x20,
        0x00, 0x41, 0x41, 0x41, 0x7F,
        0x04, 0x02, 0x01, 0x02, 0x04,
        0x40, 0x40, 0x40, 0x40, 0x40,
        0x00, 0x03, 0x07, 0x08, 0x00,
        0x20, 0x54, 0x54, 0x78, 0x40,
        0x7F, 0x28, 0x44, 0x44, 0x38,
        0x38, 0x44, 0x44, 0x44, 0x28,
        0x38, 0x44, 0x44, 0x28, 0x7F,
        0x38, 0x54, 0x54, 0x54, 0x18,
        0x00, 0x08, 0x7E, 0x09, 0x02,
        0x18, 0xA4, 0xA4, 0x9C, 0x78,
        0x7F, 0x08, 0x04, 0x04, 0x78,
        0x00, 0x44, 0x7D, 0x40, 0x00,
        0x20, 0x40, 0x40, 0x3D, 0x00,
        0x7F, 0x10, 0x28, 0x44, 0x00,
        0x00, 0x41, 0x7F, 0x40, 0x00,
        0x7C, 0x04, 0x78, 0x04, 0x78,
        0x7C, 0x08, 0x04, 0x04, 0x78,
        0x38, 0x44, 0x44, 0x44, 0x38,
        0xFC, 0x18, 0x24, 0x24, 0x18,
        0x18, 0x24, 0x24, 0x18, 0xFC,
        0x7C, 0x08, 0x04, 0x04, 0x08,
        0x48, 0x54, 0x54, 0x54, 0x24,
        0x04, 0x04, 0x3F, 0x44, 0x24,
        0x3C, 0x40, 0x40, 0x20, 0x7C,
        0x1C, 0x20, 0x40, 0x20, 0x1C,
        0x3C, 0x40, 0x30, 0x40, 0x3C,
        0x44, 0x28, 0x10, 0x28, 0x44,
        0x4C, 0x90, 0x90, 0x90, 0x7C,
        0x44, 0x64, 0x54, 0x4C, 0x44,
        0x00, 0x08, 0x36, 0x41, 0x00,
        0x00, 0x00, 0x77, 0x00, 0x00,
        0x00, 0x41, 0x36, 0x08, 0x00,
        0x02, 0x01, 0x02, 0x04, 0x02,
        0x3C, 0x26, 0x23, 0x26, 0x3C,
        0x1E, 0xA1, 0xA1, 0x61, 0x12,
        0x3A, 0x40, 0x40, 0x20, 0x7A,
        0x38, 0x54, 0x54, 0x55, 0x59,
        0x21, 0x55, 0x55, 0x79, 0x41,
        0x22, 0x54, 0x54, 0x78, 0x42, // a-umlaut
        0x21, 0x55, 0x54, 0x78, 0x40,
        0x20, 0x54, 0x55, 0x79, 0x40,
        0x0C, 0x1E, 0x52, 0x72, 0x12,
        0x39, 0x55, 0x55, 0x55, 0x59,
        0x39, 0x54, 0x54, 0x54, 0x59,
        0x39, 0x55, 0x54, 0x54, 0x58,
        0x00, 0x00, 0x45, 0x7C, 0x41,
        0x00, 0x02, 0x45, 0x7D, 0x42,
        0x00, 0x01, 0x45, 0x7C, 0x40,
        0x7D, 0x12, 0x11, 0x12, 0x7D, // A-umlaut
        0xF0, 0x28, 0x25, 0x28, 0xF0,
        0x7C, 0x54, 0x55, 0x45, 0x00,
        0x20, 0x54, 0x54, 0x7C, 0x54,
        0x7C, 0x0A, 0x09, 0x7F, 0x49,
        0x32, 0x49, 0x49, 0x49, 0x32,
        0x3A, 0x44, 0x44, 0x44, 0x3A, // o-umlaut
        0x32, 0x4A, 0x48, 0x48, 0x30,
        0x3A, 0x41, 0x41, 0x21, 0x7A,
        0x3A, 0x42, 0x40, 0x20, 0x78,
        0x00, 0x9D, 0xA0, 0xA0, 0x7D,
        0x3D, 0x42, 0x42, 0x42, 0x3D, // O-umlaut
        0x3D, 0x40, 0x40, 0x40, 0x3D,
        0x3C, 0x24, 0xFF, 0x24, 0x24,
        0x48, 0x7E, 0x49, 0x43, 0x66,
        0x2B, 0x2F, 0xFC, 0x2F, 0x2B,
        0xFF, 0x09, 0x29, 0xF6, 0x20,
        0xC0, 0x88, 0x7E, 0x09, 0x03,
        0x20, 0x54, 0x54, 0x79, 0x41,
        0x00, 0x00, 0x44, 0x7D, 0x41,
        0x30, 0x48, 0x48, 0x4A, 0x32,
        0x38, 0x40, 0x40, 0x22, 0x7A,
        0x00, 0x7A, 0x0A, 0x0A, 0x72,
        0x7D, 0x0D, 0x19, 0x31, 0x7D,
        0x26, 0x29, 0x29, 0x2F, 0x28,
        0x26, 0x29, 0x29, 0x29, 0x26,
        0x30, 0x48, 0x4D, 0x40, 0x20,
        0x38, 0x08, 0x08, 0x08, 0x08,
        0x08, 0x08, 0x08, 0x08, 0x38,
        0x2F, 0x10, 0xC8, 0xAC, 0xBA,
        0x2F, 0x10, 0x28, 0x34, 0xFA,
        0x00, 0x00, 0x7B, 0x00, 0x00,
        0x08, 0x14, 0x2A, 0x14, 0x22,
        0x22, 0x14, 0x2A, 0x14, 0x08,
        0x55, 0x00, 0x55, 0x00, 0x55, // #176 (25% block) missing in old code
        0xAA, 0x55, 0xAA, 0x55, 0xAA, // 50% block
        0xFF, 0x55, 0xFF, 0x55, 0xFF, // 75% block
        0x00, 0x00, 0x00, 0xFF, 0x00,
        0x10, 0x10, 0x10, 0xFF, 0x00,
        0x14, 0x14, 0x14, 0xFF, 0x00,
        0x10, 0x10, 0xFF, 0x00, 0xFF,
        0x10, 0x10, 0xF0, 0x10, 0xF0,
        0x14, 0x14, 0x14, 0xFC, 0x00,
        0x14, 0x14, 0xF7, 0x00, 0xFF,
        0x00, 0x00, 0xFF, 0x00, 0xFF,
        0x14, 0x14, 0xF4, 0x04, 0xFC,
        0x14, 0x14, 0x17, 0x10, 0x1F,
        0x10, 0x10, 0x1F, 0x10, 0x1F,
        0x14, 0x14, 0x14, 0x1F, 0x00,
        0x10, 0x10, 0x10, 0xF0, 0x00,
        0x00, 0x00, 0x00, 0x1F, 0x10,
        0x10, 0x10, 0x10, 0x1F, 0x10,
        0x10, 0x10, 0x10, 0xF0, 0x10,
        0x00, 0x00, 0x00, 0xFF, 0x10,
        0x10, 0x10, 0x10, 0x10, 0x10,
        0x10, 0x10, 0x10, 0xFF, 0x10,
        0x00, 0x00, 0x00, 0xFF, 0x14,
        0x00, 0x00, 0xFF, 0x00, 0xFF,
        0x00, 0x00, 0x1F, 0x10, 0x17,
        0x00, 0x00, 0xFC, 0x04, 0xF4,
        0x14, 0x14, 0x17, 0x10, 0x17,
        0x14, 0x14, 0xF4, 0x04, 0xF4,
        0x00, 0x00, 0xFF, 0x00, 0xF7,
        0x14, 0x14, 0x14, 0x14, 0x14,
        0x14, 0x14, 0xF7, 0x00, 0xF7,
        0x14, 0x14, 0x14, 0x17, 0x14,
        0x10, 0x10, 0x1F, 0x10, 0x1F,
        0x14, 0x14, 0x14, 0xF4, 0x14,
        0x10, 0x10, 0xF0, 0x10, 0xF0,
        0x00, 0x00, 0x1F, 0x10, 0x1F,
        0x00, 0x00, 0x00, 0x1F, 0x14,
        0x00, 0x00, 0x00, 0xFC, 0x14,
        0x00, 0x00, 0xF0, 0x10, 0xF0,
        0x10, 0x10, 0xFF, 0x10, 0xFF,
        0x14, 0x14, 0x14, 0xFF, 0x14,
        0x10, 0x10, 0x10, 0x1F, 0x00,
        0x00, 0x00, 0x00, 0xF0, 0x10,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
        0xFF, 0xFF, 0xFF, 0x00, 0x00,
        0x00, 0x00, 0x00, 0xFF, 0xFF,
        0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
        0x38, 0x44, 0x44, 0x38, 0x44,
        0xFC, 0x4A, 0x4A, 0x4A, 0x34, // sharp-s or beta
        0x7E, 0x02, 0x02, 0x06, 0x06,
        0x02, 0x7E, 0x02, 0x7E, 0x02,
        0x63, 0x55, 0x49, 0x41, 0x63,
        0x38, 0x44, 0x44, 0x3C, 0x04,
        0x40, 0x7E, 0x20, 0x1E, 0x20,
        0x06, 0x02, 0x7E, 0x02, 0x02,
        0x99, 0xA5, 0xE7, 0xA5, 0x99,
        0x1C, 0x2A, 0x49, 0x2A, 0x1C,
        0x4C, 0x72, 0x01, 0x72, 0x4C,
        0x30, 0x4A, 0x4D, 0x4D, 0x30,
        0x30, 0x48, 0x78, 0x48, 0x30,
        0xBC, 0x62, 0x5A, 0x46, 0x3D,
        0x3E, 0x49, 0x49, 0x49, 0x00,
        0x7E, 0x01, 0x01, 0x01, 0x7E,
        0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
        0x44, 0x44, 0x5F, 0x44, 0x44,
        0x40, 0x51, 0x4A, 0x44, 0x40,
        0x40, 0x44, 0x4A, 0x51, 0x40,
        0x00, 0x00, 0xFF, 0x01, 0x03,
        0xE0, 0x80, 0xFF, 0x00, 0x00,
        0x08, 0x08, 0x6B, 0x6B, 0x08,
        0x36, 0x12, 0x36, 0x24, 0x36,
        0x06, 0x0F, 0x09, 0x0F, 0x06,
        0x00, 0x00, 0x18, 0x18, 0x00,
        0x00, 0x00, 0x10, 0x10, 0x00,
        0x30, 0x40, 0xFF, 0x01, 0x01,
        0x00, 0x1F, 0x01, 0x01, 0x1E,
        0x00, 0x19, 0x1D, 0x17, 0x12,
        0x00, 0x3C, 0x3C, 0x3C, 0x3C,
        0x00, 0x00, 0x00, 0x00, 0x00 // #255 NBSP
};

typedef uint16_t color_t; // type used for colors (unsigned)

// swap any type
template <typename T>
static inline __attribute__((always_inline)) void swapValue(T &x, T &y) {
    T tmp = x;
    x = y;
    y = tmp;
}

// minimum value for any type
template <typename T>
static inline __attribute__((always_inline)) T minValue(T &x, T &y) {
    return x < y ? x : y;
}

// maximum value for any type
template <typename T>
static inline __attribute__((always_inline)) T maxValue(T &x, T &y) {
    return x >= y ? x : y;
}

#define PARENT_TEMPLATE_PARAMS class HW, class coord_t, class s_coord_t, coord_t WIDTH, coord_t HEIGHT
#define PARENT_TEMPLATE_PARAM_NAMES HW, coord_t, s_coord_t, WIDTH, HEIGHT
#define PARENT_TEMPLATE_DEF template <PARENT_TEMPLATE_PARAMS>

PARENT_TEMPLATE_DEF
class PDQ_GFX : public Print {

  public:
    // Graphic primitives
    // drawPixel MUST be defined by the driver subclass (and has no generic fall-back):

    // These are generic versions of routines for drivers that don't provide device-optimized code.
    // Drivers are required to have these functions (without "_" postfix), but can fall back to using
    // these if needed (they should not be called directly with "_" postfix or it will bypass any
    // device-optimized implementations).
    static void drawLine_(coord_t x0, coord_t y0, coord_t x1, coord_t y1, color_t color);
    static void drawFastVLine_(coord_t x, coord_t y, coord_t h, color_t color);
    static void drawFastHLine_(coord_t x, coord_t y, coord_t w, color_t color);
    static void fillRect_(coord_t x, coord_t y, coord_t w, coord_t h, color_t color);
    static void fillScreen_(color_t color);

    // These are usually overridden in the driver subclass to be useful (but not internally referenced)
    static void setRotation(uint8_t r); // only swaps width/height if not supported by driver
    static void invertDisplay(bool i);  // only if supported by driver

    // These exist in PDQ_GFX (and generally have no subclass override)
    static void drawRect(s_coord_t x, s_coord_t y, coord_t w, coord_t h, color_t color);
    static void drawCircle(coord_t x0, coord_t y0, coord_t r, color_t color);
    static void drawCircleHelper(coord_t x0, coord_t y0, coord_t r, uint8_t cornername, color_t color);
    static void fillCircle(coord_t x0, coord_t y0, coord_t r, color_t color);
    static void fillCircleHelper(coord_t x0, coord_t y0, coord_t r, uint8_t cornername, coord_t delta, color_t color);
    static void drawTriangle(coord_t x0, coord_t y0, coord_t x1, coord_t y1, coord_t x2, coord_t y2, color_t color);
    static void fillTriangle(coord_t x0, coord_t y0, coord_t x1, coord_t y1, coord_t x2, coord_t y2, color_t color);
    static void drawRoundRect(coord_t x0, coord_t y0, coord_t w, coord_t h, coord_t radius, color_t color);
    static void fillRoundRect(coord_t x0, coord_t y0, coord_t w, coord_t h, coord_t radius, color_t color);
    static inline void drawBitmap(coord_t x, coord_t y, const uint8_t *bitmap, coord_t w, coord_t h, color_t color);
    static inline void drawBitmap(coord_t x, coord_t y, const uint8_t *bitmap, coord_t w, coord_t h, color_t color, color_t bg);
    static inline void drawBitmap(coord_t x, coord_t y, uint8_t *bitmap, coord_t w, coord_t h, color_t color);
    static inline void drawBitmap(coord_t x, coord_t y, uint8_t *bitmap, coord_t w, coord_t h, color_t color, color_t bg);
    static void drawXBitmap(coord_t x, coord_t y, const uint8_t *bitmap, coord_t w, coord_t h, color_t color);
    static void drawChar(coord_t x, coord_t y, unsigned char c, color_t color, color_t bg);
    static inline void setCursor(coord_t x, coord_t y);
    static inline void setTextColor(color_t c);
    static inline void setTextColor(color_t c, color_t bg);
    static inline void setTextSize(uint8_t s);
    static inline void setTextWrap(bool w);

    static inline coord_t width() __attribute__((always_inline)) { return _width; }
    static inline coord_t height() __attribute__((always_inline)) { return _height; }
    static inline uint8_t getRotation() __attribute__((always_inline)) { return rotation; }
    static inline coord_t getCursorX() __attribute__((always_inline)) { return cursor_x; }
    static inline coord_t getCursorY() __attribute__((always_inline)) { return cursor_y; }
    static inline void getTextBounds(const char *__restrict__ string, coord_t x, coord_t y, int16_t *__restrict__ x1, int16_t *__restrict__ y1, uint16_t *__restrict__ w, uint16_t *__restrict__ h);
    static inline void getTextBounds(const __FlashStringHelper *__restrict__ s, coord_t x, coord_t y, int16_t *__restrict__ x1, int16_t *__restrict__ y1, uint16_t *__restrict__ w, uint16_t *__restrict__ h);

    virtual size_t write(uint8_t); // used by Arduino "Print.h" (and the one required virtual function)

  private:
    static void generalGetTextBounds(uint8_t (*memReader)(const uint8_t *), const uint8_t *__restrict__ str, coord_t x, coord_t y, int16_t *__restrict__ x1, int16_t *__restrict__ y1, uint16_t *__restrict__ w, uint16_t *__restrict__ h);
    static void generalDrawBitmap(uint8_t (*memReader)(const uint8_t *), coord_t x, coord_t y, uint8_t *bitmap, coord_t w, coord_t h, color_t color, bool setBg, color_t bg);

  protected:
    static coord_t _width, _height; // Display w/h as modified by current rotation
    static coord_t cursor_x, cursor_y;
    static color_t textcolor, textbgcolor;
    static uint8_t textsize;
    static uint8_t rotation;
    static bool wrap; // If set, 'wrap' text at right edge of display
};

PARENT_TEMPLATE_DEF
class PDQ_GFX_Button_ {
  public:
    void initButton(PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES> *gfx, coord_t x, coord_t y, coord_t w, coord_t h, color_t outline, color_t fill, color_t textcolor, const char *label, uint8_t textsize);
    void drawButton(bool inverted = false);
    bool contains(coord_t x, coord_t y);

    void press(bool p);
    bool isPressed();
    bool justPressed();
    bool justReleased();

  private:
    PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES> *_gfx = NULL;
    int16_t _x, _y;
    int16_t _w, _h;
    uint8_t _textsize;
    color_t _outlinecolor, _fillcolor, _textcolor;
    char _label[10];

    bool currstate, laststate;
};

// -----------------------------------------------

static inline uint8_t normalMemReader(const uint8_t *str) {
    return *str;
}

static inline uint8_t progmemReader(const uint8_t *str) {
    return pgm_read_byte(str);
}

PARENT_TEMPLATE_DEF
coord_t PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::_width = WIDTH; // Display w/h as modified by current rotation
PARENT_TEMPLATE_DEF
coord_t PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::_height = HEIGHT;
PARENT_TEMPLATE_DEF
coord_t PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::cursor_x = 0;
PARENT_TEMPLATE_DEF
coord_t PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::cursor_y = 0;
PARENT_TEMPLATE_DEF
color_t PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::textcolor = 0xffff;
PARENT_TEMPLATE_DEF
color_t PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::textbgcolor = 0xffff;
PARENT_TEMPLATE_DEF
uint8_t PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::textsize = 1;
PARENT_TEMPLATE_DEF
uint8_t PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::rotation = 0;
PARENT_TEMPLATE_DEF
bool PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::wrap = true; // If set, 'wrap' text at right edge of display

// Draw a circle outline
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawCircle(coord_t x0, coord_t y0, coord_t r, color_t color) {
    HW::drawPixel(x0, y0 + r, color);
    HW::drawPixel(x0, y0 - r, color);
    HW::drawPixel(x0 + r, y0, color);
    HW::drawPixel(x0 - r, y0, color);

    drawCircleHelper(x0, y0, r, 0xFF, color);
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawCircleHelper(coord_t x0, coord_t y0, coord_t r, uint8_t cornername, color_t color) {
    s_coord_t f = 1 - r;
    coord_t ddF_x = 1;
    s_coord_t ddF_y = -(r << 1);
    coord_t x = 0;
    coord_t y = r;

    while (x < y) {
        if (f >= 0) {
            --y;
            ddF_y += 2;
            f += ddF_y;
        }
        ++x;
        ddF_x += 2;
        f += ddF_x;

        if (cornername & 0x4) {
            HW::drawPixel(x0 + x, y0 + y, color);
            HW::drawPixel(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
            HW::drawPixel(x0 + x, y0 - y, color);
            HW::drawPixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
            HW::drawPixel(x0 - y, y0 + x, color);
            HW::drawPixel(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
            HW::drawPixel(x0 - y, y0 - x, color);
            HW::drawPixel(x0 - x, y0 - y, color);
        }
    }
}

// Used to do circles and roundrects
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::fillCircleHelper(coord_t x0, coord_t y0, coord_t r, uint8_t cornername, coord_t delta, color_t color) {
    s_coord_t f = 1 - r;
    coord_t ddF_x = 1;
    s_coord_t ddF_y = -(r << 1);
    coord_t x = 0;
    coord_t y = r;

    while (x < y) {
        if (f >= 0) {
            --y;
            ddF_y += 2;
            f += ddF_y;
        }
        ++x;
        ddF_x += 2;
        f += ddF_x;

        if (cornername & 0x1) {
            HW::drawFastVLine(x0 + x, y0 - y, 2 * y + 1 + delta, color);
            HW::drawFastVLine(x0 + y, y0 - x, 2 * x + 1 + delta, color);
        }
        if (cornername & 0x2) {
            HW::drawFastVLine(x0 - x, y0 - y, 2 * y + 1 + delta, color);
            HW::drawFastVLine(x0 - y, y0 - x, 2 * x + 1 + delta, color);
        }
    }
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::fillCircle(coord_t x0, coord_t y0, coord_t r, color_t color) {
    HW::drawFastVLine(x0, y0 - r, 2 * r + 1, color);
    fillCircleHelper(x0, y0, r, 3, 0, color);
}

// Bresenham's algorithm - thx Wikipedia
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawLine_(coord_t x0, coord_t y0, coord_t x1, coord_t y1, color_t color) {
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        swapValue(x0, y0);
        swapValue(x1, y1);
    }

    if (x0 > x1) {
        swapValue(x0, x1);
        swapValue(y0, y1);
    }

    coord_t dx = x1 - x0;
    coord_t dy = abs(y1 - y0);

    s_coord_t err = dx / 2;
    int8_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0 <= x1; ++x0) {
        if (steep) {
            HW::drawPixel(y0, x0, color);
        } else {
            HW::drawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

// Draw a rectangle
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawRect(s_coord_t x, s_coord_t y, coord_t w, coord_t h, color_t color) {
    HW::drawFastHLine(x, y, w, color);
    HW::drawFastHLine(x, y + h - 1, w, color);
    HW::drawFastVLine(x, y, h, color);
    HW::drawFastVLine(x + w - 1, y, h, color);
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawFastVLine_(coord_t x, coord_t y, coord_t h, color_t color) {
    // Used by driver when it has no special support
    HW::drawLine(x, y, x, y + h - 1, color);
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawFastHLine_(coord_t x, coord_t y, coord_t w, color_t color) {
    // Used by driver when it has no special support
    HW::drawLine(x, y, x + w - 1, y, color);
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::fillRect_(coord_t x, coord_t y, coord_t w, coord_t h, color_t color) {
    // Used by driver when it has no special support
    for (coord_t i = x; i < x + w; ++i) {
        HW::drawFastVLine(i, y, h, color);
    }
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::fillScreen_(color_t color) {
    // Used by driver when it has no special support
    HW::fillRect(0, 0, _width, _height, color);
}

// Draw a rounded rectangle
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawRoundRect(coord_t x, coord_t y, coord_t w, coord_t h, coord_t r, color_t color) {
    // smarter version
    // draw four round corners + 4 rects to form the round rect
    const coord_t truncWidth = w - (r << 1);
    const coord_t truncHeight = h - (r << 1);
    const coord_t topYoffset = y + r;
    const coord_t leftXoffset = x + r;
    HW::drawFastHLine(leftXoffset, y, truncWidth, color); // Top
    coord_t bottomYoffset = y + h - 1;
    HW::drawFastHLine(leftXoffset, bottomYoffset, truncWidth, color); // Bottom
    bottomYoffset -= r;
    drawCircleHelper(leftXoffset, bottomYoffset, r, 8, color);
    drawCircleHelper(leftXoffset, topYoffset, r, 1, color);
    HW::drawFastVLine(x, topYoffset, truncHeight, color); // Left
    coord_t rightOffset = x + w - 1;
    HW::drawFastVLine(rightOffset, topYoffset, truncHeight, color); // Right
    rightOffset -= r;
    drawCircleHelper(rightOffset, topYoffset, r, 2, color);
    drawCircleHelper(rightOffset, bottomYoffset, r, 4, color);
}

// Fill a rounded rectangle
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::fillRoundRect(coord_t x, coord_t y, coord_t w, coord_t h, coord_t r, color_t color) {
    // smarter version
    const coord_t leftXoffset = x + r;
    HW::fillRect(leftXoffset, y, w - (r << 1), h, color);

    // draw four corners
    const coord_t delta = h - (r << 1) - 1;
    const coord_t topYoffset = y + r;
    fillCircleHelper(leftXoffset, topYoffset, r, 2, delta, color);
    fillCircleHelper(x + w - r - 1, topYoffset, r, 1, delta, color);
}

// Draw a triangle
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawTriangle(coord_t x0, coord_t y0, coord_t x1, coord_t y1, coord_t x2, coord_t y2, color_t color) {
    HW::drawLine(x0, y0, x1, y1, color);
    HW::drawLine(x1, y1, x2, y2, color);
    HW::drawLine(x2, y2, x0, y0, color);
}

// Fill a triangle
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::fillTriangle(coord_t x0, coord_t y0, coord_t x1, coord_t y1, coord_t x2, coord_t y2, color_t color) {

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
        swapValue(y0, y1);
        swapValue(x0, x1);
    }
    if (y1 > y2) {
        swapValue(y2, y1);
        swapValue(x2, x1);
    }
    if (y0 > y1) {
        swapValue(y0, y1);
        swapValue(x0, x1);
    }

    // Handle awkward all-on-same-line case as its own thing
    if (y0 == y2) {
        coord_t a = x0;
        coord_t b = x0;
        if (x1 < a) {
            a = x1;
        } else {
            b = x1;
        }

        if (x2 < a) {
            a = x2;
        } else if (x2 > b) {
            b = x2;
        }
        HW::drawFastHLine(a, y0, b - a + 1, color);
        return;
    }

    const s_coord_t dx01 = x1 - x0;
    const s_coord_t dx02 = x2 - x0;
    const s_coord_t dx12 = x2 - x1;
    const coord_t dy01 = y1 - y0;
    const coord_t dy02 = y2 - y0;
    const coord_t dy12 = y2 - y1;
    //TODO for ST7735 this is enough for bigger displays you might need 32 bit instead of 16
    // you need space for the highest possible value: width*height
    s_coord_t sa = 0;
    s_coord_t sb = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.	If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    coord_t last = y1;
    // Check if we need to include y1 scanline
    if (y1 != y2) {
        last -= 1; // Skip it
    }

    coord_t y = y0;
    for (; y <= last; ++y) {
        s_coord_t a = sa / dy01;
        s_coord_t b = sb / dy02;
        /* longhand:
		a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
		b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		*/
        if (a > b)
            swapValue(a, b);
        HW::drawFastHLine(a + x0, y, b - a + 1, color);

        sa += dx01;
        sb += dx02;
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.	This loop is skipped if y1=y2.
    sa = y1 != y2 ? 0 : dx12;
    sb = dx02 * (y - y0);
    for (; y <= y2; ++y) {
        coord_t a = x1 + sa / dy12;
        coord_t b = x0 + sb / dy02;
        /* longhand:
		a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
		b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		*/
        if (a > b)
            swapValue(a, b);
        HW::drawFastHLine(a, y, b - a + 1, color);

        sa += dx12;
        sb += dx02;
    }
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::generalDrawBitmap(uint8_t (*memReader)(const uint8_t *), coord_t x, coord_t y, uint8_t *bitmap, coord_t w, coord_t h, color_t color, bool setBg, color_t bg) {
    coord_t i, j, byteWidth = (w + 7) / 8;
    uint8_t byte;

    for (j = 0; j < h; ++j) {
        for (i = 0; i < w; ++i) {
            if (i % 8 == 0)
                byte = memReader(bitmap + j * byteWidth + i / 8);
            else
                byte <<= 1;

            if (byte & 0x80) {
                HW::drawPixel(x + i, y + j, color);
            } else if (setBg) {
                HW::drawPixel(x + i, y + j, bg);
            }
        }
    }
}

// Draw a 1-bit image (bitmap) at the specified (x, y) position from the
// provided bitmap buffer (must be PROGMEM memory) using the specified
// foreground (for set bits) and background (for clear bits) colors.
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawBitmap(coord_t x, coord_t y, const uint8_t *bitmap, coord_t w, coord_t h, color_t color, color_t bg) {
    generalDrawBitmap(progmemReader, x, y, bitmap, w, h, color, true, bg);
}

// drawBitmap() variant w/background for RAM-resident (not PROGMEM) bitmaps.
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawBitmap(coord_t x, coord_t y, uint8_t *bitmap, coord_t w, coord_t h, color_t color, color_t bg) {
    generalDrawBitmap(normalMemReader, x, y, bitmap, w, h, color, true, bg);
}

// Draw a 1-bit image (bitmap) at the specified (x, y) position from the
// provided bitmap buffer (must be PROGMEM memory) using the specified
// foreground color (unset bits are transparent).
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawBitmap(coord_t x, coord_t y, const uint8_t *bitmap, coord_t w, coord_t h, color_t color) {
    generalDrawBitmap(progmemReader, x, y, bitmap, w, h, color, false, 0);
}

// drawBitmap() variant for RAM-resident (not PROGMEM) bitmaps.
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawBitmap(coord_t x, coord_t y, uint8_t *bitmap, coord_t w, coord_t h, color_t color) {
    generalDrawBitmap(normalMemReader, x, y, bitmap, w, h, color, false, 0);
}

// Draw XBitMap Files (*.xbm), exported from GIMP,
// Usage: Export from GIMP to *.xbm, rename *.xbm to *.c and open in editor.
// C Array can be directly used with this function
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawXBitmap(coord_t x, coord_t y, const uint8_t *bitmap, coord_t w, coord_t h, color_t color) {
    coord_t i, j, byteWidth = (w + 7) / 8;
    uint8_t byte;

    for (j = 0; j < h; ++j) {
        for (i = 0; i < w; ++i) {
            if (i % 8 == 0)
                byte = pgm_read_byte(bitmap + j * byteWidth + i / 8);
            else
                byte >>= 1;

            if (byte & 0x01)
                HW::drawPixel(x + i, y + j, color);
        }
    }
}

PARENT_TEMPLATE_DEF
size_t PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::write(uint8_t c) {
    // 'Classic' built-in font
    if (c == '\n') {
        cursor_x = 0;
        cursor_y += (coord_t)textsize * Y_PIXEL_PER_CHAR;
    } else if (c != '\r') {
        HW::drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor);
        cursor_x += textsize * X_PIXEL_PER_CHAR;
        if (wrap && (cursor_x > (_width - textsize * X_PIXEL_PER_CHAR))) {
            cursor_x = 0;
            cursor_y += textsize * Y_PIXEL_PER_CHAR;
        }
    }
    return 1;
}

// Draw a character with built-in font
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::drawChar(coord_t x, coord_t y, unsigned char c, color_t color, color_t bg) {
    if ((x >= _width) ||
        (y >= _height) ||
        ((x + (X_PIXEL_PER_CHAR * textsize) - 1) < 0) ||
        ((y + (Y_PIXEL_PER_CHAR * textsize) - 1) < 0)) {
        return;
    }

    bool is_opaque = (bg != color);
    // Handle 'classic' charset behavior
    if (c >= 176) {
        ++c;
    }

    const uint8_t *readAddress = glcdfont + c * 5;

    const bool isTextSize1 = textsize == 1;

    const coord_t endIt = y + (textsize << 3);
    for (uint8_t i = 0; i < X_PIXEL_PER_CHAR; ++i, x += textsize) {
        uint8_t line;

        if (i == 5)
            line = 0x00;
        else
            line = pgm_read_byte(readAddress++);

        for (coord_t j = y; j < endIt; j += textsize) {
            if (line & 0x01) {
                if (isTextSize1) {
                    HW::drawPixel(x, j, color);
                } else {
                    HW::fillRect(x, j, textsize, textsize, color);
                }
            } else if (is_opaque) {
                if (isTextSize1) {
                    HW::drawPixel(x, j, bg);
                } else {
                    HW::fillRect(x, j, textsize, textsize, bg);
                }
            }
            line >>= 1;
        }
    }
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::setCursor(coord_t x, coord_t y) {
    cursor_x = x;
    cursor_y = y;
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::setTextSize(uint8_t s) {
    textsize = (s > 0) ? s : 1;
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::setTextColor(color_t c) {
    // For 'transparent' background, we'll set the bg
    // to the same as fg instead of using a flag
    textcolor = c;
    textbgcolor = c;
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::setTextColor(color_t c, color_t b) {
    textcolor = c;
    textbgcolor = b;
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::setTextWrap(bool w) {
    wrap = w;
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::setRotation(uint8_t x) {
    // Used by driver when it has no special support
    rotation = x & 3;
    switch (rotation) {
        case 0:
        case 2:
            _width = WIDTH;
            _height = HEIGHT;
            break;
        case 1:
        case 3:
            _width = HEIGHT;
            _height = WIDTH;
            break;
    }
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::generalGetTextBounds(uint8_t (*strReader)(const uint8_t *), const uint8_t *__restrict__ str, coord_t x, coord_t y, int16_t *__restrict__ x1, int16_t *__restrict__ y1, uint16_t *__restrict__ w, uint16_t *__restrict__ h) {
    uint8_t c; // Current character

    *x1 = x;
    *y1 = y;
    *w = *h = 0;

    // Default font
    uint16_t lineWidth = 0, maxWidth = 0; // Width of current, all lines

    while ((c = strReader(str++))) {
        // Not a newline
        if (c != '\n') {
            // Not a carriage return, is normal char
            if (c != '\r') {
                // Includes interchar x gap
                lineWidth += textsize * X_PIXEL_PER_CHAR;
                //TODO check which one is correct... progmem version has different condition:
                // if (wrap && ((x + textsize * X_PIXEL_PER_CHAR) >= _width)) {
                if (wrap && (cursor_x > (_width - textsize * X_PIXEL_PER_CHAR))) {
                    x = 0;
                    y += textsize * Y_PIXEL_PER_CHAR;
                    // Save widest line
                    if (lineWidth > maxWidth) {
                        maxWidth = lineWidth;
                    }
                    lineWidth = textsize * X_PIXEL_PER_CHAR; // First char on new line
                }
            } // Carriage return = do nothing
        } else {
            // Newline
            x = 0;                            // Reset x to 0
            y += textsize * Y_PIXEL_PER_CHAR; // Advance y by 1 line
            // Save widest line
            if (lineWidth > maxWidth) {
                maxWidth = lineWidth;
            }
            lineWidth = 0; // Reset lineWidth for new line
        }
    }
    // End of string
    // Add height of last (or only) line
    if (lineWidth) {
        y += textsize * Y_PIXEL_PER_CHAR;
    }
    // Save widest line
    if (lineWidth > maxWidth) {
        maxWidth = lineWidth;
    }
    *w = maxWidth - 1; // Don't include last interchar x gap
    *h = y - *y1;
}

// Pass string and a cursor position, returns UL corner and W,H.
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::getTextBounds(const char *str, coord_t x, coord_t y, int16_t *__restrict__ x1, int16_t *__restrict__ y1, uint16_t *__restrict__ w, uint16_t *__restrict__ h) {
    generalGetTextBounds(normalMemReader, (const uint8_t *)str, x, y, x1, y1, w, h);
}

// Same as above, but for PROGMEM strings
PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::getTextBounds(const __FlashStringHelper *str, coord_t x, coord_t y, int16_t *__restrict__ x1, int16_t *__restrict__ y1, uint16_t *__restrict__ w, uint16_t *__restrict__ h) {
    generalGetTextBounds(progmemReader, (const uint8_t *)str, x, y, x1, y1, w, h);
}

PARENT_TEMPLATE_DEF
void PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES>::invertDisplay(bool i) {
    // Used by driver when it has no special support
    // Do nothing, must be supported by driver
}

/***************************************************************************/
// code for the GFX button UI element

PARENT_TEMPLATE_DEF
void PDQ_GFX_Button_<PARENT_TEMPLATE_PARAM_NAMES>::initButton(PDQ_GFX<PARENT_TEMPLATE_PARAM_NAMES> *gfx, coord_t x, coord_t y, coord_t w, coord_t h, color_t outline, color_t fill, color_t textcolor, const char *label, uint8_t textsize) {
    _gfx = gfx;
    _x = x;
    _y = y;
    _w = w;
    _h = h;
    _outlinecolor = outline;
    _fillcolor = fill;
    _textcolor = textcolor;
    _textsize = textsize;
    strncpy(_label, label, 9);
    _label[9] = 0;
}

PARENT_TEMPLATE_DEF
void PDQ_GFX_Button_<PARENT_TEMPLATE_PARAM_NAMES>::drawButton(bool inverted) {
    color_t fill, outline, text;

    if (!inverted) {
        fill = _fillcolor;
        outline = _outlinecolor;
        text = _textcolor;
    } else {
        fill = _textcolor;
        outline = _outlinecolor;
        text = _fillcolor;
    }

    _gfx->fillRoundRect(_x - (_w / 2), _y - (_h / 2), _w, _h, min(_w, _h) / 4, fill);
    _gfx->drawRoundRect(_x - (_w / 2), _y - (_h / 2), _w, _h, min(_w, _h) / 4, outline);

    _gfx->setCursor(_x - strlen(_label) * 3 * _textsize, _y - 4 * _textsize);
    _gfx->setTextColor(text);
    _gfx->setTextSize(_textsize);
    _gfx->print(_label);
}

PARENT_TEMPLATE_DEF
bool PDQ_GFX_Button_<PARENT_TEMPLATE_PARAM_NAMES>::contains(coord_t x, coord_t y) {
    if ((x < (_x - _w / 2)) || (x > (_x + _w / 2)))
        return false;
    if ((y < (_y - _h / 2)) || (y > (_y + _h / 2)))
        return false;
    return true;
}

PARENT_TEMPLATE_DEF
void PDQ_GFX_Button_<PARENT_TEMPLATE_PARAM_NAMES>::press(bool p) {
    laststate = currstate;
    currstate = p;
}

PARENT_TEMPLATE_DEF
bool PDQ_GFX_Button_<PARENT_TEMPLATE_PARAM_NAMES>::isPressed() {
    return currstate;
}

PARENT_TEMPLATE_DEF
bool PDQ_GFX_Button_<PARENT_TEMPLATE_PARAM_NAMES>::justPressed() {
    return (currstate && !laststate);
}

PARENT_TEMPLATE_DEF
bool PDQ_GFX_Button_<PARENT_TEMPLATE_PARAM_NAMES>::justReleased() {
    return (!currstate && laststate);
}

#undef PARENT_TEMPLATE_PARAMS
#undef PARENT_TEMPLATE_PARAM_NAMES
#undef PARENT_TEMPLATE_DEF

#endif // _PDQ_GFX_H
