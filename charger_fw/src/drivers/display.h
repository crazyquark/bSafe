#pragma once
// =============================================================================
// display.h — framebuffer + text rendering over esp_lcd SSD1306 panel
//
// Uses font8x8_basic.h (ASCII 0x00-0x7F, 8x8 pixels per glyph).
// Framebuffer is 128x64 / 8 = 1024 bytes, flushed in one DMA transfer.
//
// Coordinate system: x = 0..127 (left→right), y = 0..63 (top→bottom)
// Text grid:         col = 0..15, row = 0..7  (each cell = 8x8 px)
// =============================================================================

#include <stdint.h>
#include <stdbool.h>
#include "esp_lcd_panel_ops.h"

#define DISP_W          128
#define DISP_H          64
#define DISP_PAGES      (DISP_H / 8)           // 8
#define DISP_FB_BYTES   (DISP_W * DISP_PAGES)  // 1024
#define DISP_COLS       (DISP_W / 8)           // 16 text columns
#define DISP_ROWS       DISP_PAGES             // 8 text rows

typedef struct {
    esp_lcd_panel_handle_t panel;
    uint8_t fb[DISP_FB_BYTES];  // 1-bit framebuffer, SSD1306 page layout
} display_t;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

/** @brief Attach to an already-initialised esp_lcd panel handle. Clears screen. */
void display_init(display_t *d, esp_lcd_panel_handle_t panel);

/** @brief Push framebuffer to hardware. Call after any draw operations. */
void display_flush(display_t *d);

// ---------------------------------------------------------------------------
// Framebuffer primitives
// ---------------------------------------------------------------------------

/** @brief Fill entire framebuffer (0x00 = black, 0xFF = white). */
void display_clear(display_t *d, uint8_t fill);

/** @brief Set/clear a single pixel. */
void display_set_pixel(display_t *d, int x, int y, bool on);

/** @brief Draw a horizontal line. */
void display_hline(display_t *d, int x, int y, int w, bool on);

/** @brief Draw a vertical line. */
void display_vline(display_t *d, int x, int y, int h, bool on);

/** @brief Draw a rectangle outline. */
void display_rect(display_t *d, int x, int y, int w, int h, bool on);

/** @brief Draw a filled rectangle. */
void display_fill_rect(display_t *d, int x, int y, int w, int h, bool on);

// ---------------------------------------------------------------------------
// Text rendering (font8x8_basic.h glyphs)
// ---------------------------------------------------------------------------

/**
 * @brief Draw one ASCII character at pixel position (x, y).
 *        Clips silently if out of bounds.
 * @param inv  true = white text on black, false = black text on white
 */
void display_draw_char(display_t *d, int x, int y, char c, bool inv);

/**
 * @brief Draw a null-terminated string starting at pixel (x, y).
 *        Does not wrap. Clips at right edge.
 */
void display_draw_str(display_t *d, int x, int y, const char *s, bool inv);

/**
 * @brief Draw string at text-grid position (col 0..15, row 0..7).
 *        Convenience wrapper around display_draw_str.
 */
void display_text(display_t *d, int col, int row, const char *s, bool inv);

/**
 * @brief Draw string scaled 2x (16x16 per glyph).
 *        Max 8 chars per row, 4 rows available.
 */
void display_draw_str_2x(display_t *d, int x, int y, const char *s, bool inv);

// ---------------------------------------------------------------------------
// Higher-level helpers matching the MicroPython UI patterns
// ---------------------------------------------------------------------------

/** @brief Draw a full-width status bar on row 0 with label left, value right. */
void display_status_bar(display_t *d, const char *label, const char *value);

/** @brief Draw a labelled value line: "LABEL   value_str" at given row. */
void display_data_row(display_t *d, int row, const char *label, const char *value);

/** @brief Draw a progress/charge bar at given row, 0..100 percent. */
void display_bar(display_t *d, int row, int percent);

/** @brief Invert a full 8px-tall text row. */
void display_invert_row(display_t *d, int row);
