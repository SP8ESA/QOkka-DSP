// ssd1106.h - SSD1306 OLED driver with DMA for Raspberry Pi Pico
// 128x64 monochrome, I2C, zero-CPU display transfer via DMA

#ifndef SSD1106_H
#define SSD1106_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

#define SSD1106_WIDTH   128
#define SSD1106_HEIGHT  64
#define SSD1106_PAGES   (SSD1106_HEIGHT / 8)
#define SSD1106_ADDR    0x3C

// Initialize display + claim DMA channel
void ssd1106_init(i2c_inst_t *i2c);

// Clear the framebuffer
void ssd1106_clear(void);

// Clear only specific pages (0-based, inclusive)
void ssd1106_clear_pages(int page_start, int page_end);

// Send framebuffer to display via DMA (returns immediately, zero CPU)
// Returns false if previous DMA transfer is still in progress.
bool ssd1106_display_dma(i2c_inst_t *i2c);

// Check if DMA transfer is still running
bool ssd1106_dma_busy(void);

// Blocking display (used only during init)
void ssd1106_display(i2c_inst_t *i2c);

// Draw a character at (x, page) using 5x7 font
void ssd1106_draw_char(int x, int page, char c);

// Draw a string at (x, page)
void ssd1106_draw_string(int x, int page, const char *str);

// Draw a string with bold (faux-bold, 6x7, 7px per char)
void ssd1106_draw_string_bold(int x, int page, const char *str);

// Draw a string with bold at arbitrary pixel Y position (not page-aligned)
void ssd1106_draw_string_bold_y(int x, int y, const char *str);

// Draw a bold string at pixel Y — inverted (white bg, black text), field_w = background width in px
void ssd1106_draw_string_bold_y_inv(int x, int y, const char *str, int field_w);

// Draw a string with 2x scale (128x64 pixels per char)
void ssd1106_draw_string_2x(int x, int page, const char *str);

// Set a pixel in the buffer
void ssd1106_set_pixel(int x, int y, bool on);

// Draw a horizontal line
void ssd1106_hline(int x0, int x1, int y);

// Draw a vertical line
void ssd1106_vline(int x, int y0, int y1);

// Fill a rectangular region
void ssd1106_fill_rect(int x0, int y0, int x1, int y1);

// Scroll a rectangular pixel region left by 1 pixel (for waterfall)
void ssd1106_scroll_region_left(int x0, int y0, int x1, int y1);

// Draw a vertical bar in the waterfall
void ssd1106_draw_wf_column(int x, int y_top, int y_bot, uint8_t level_0_8);

// Get pointer to framebuffer (for direct manipulation)
uint8_t *ssd1106_get_framebuf(void);

#endif // SSD1106_H
