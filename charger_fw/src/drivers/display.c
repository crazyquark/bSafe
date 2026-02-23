// Suppress unused-variable warning — we include the header for font8x8_basic_tr only
#pragma GCC diagnostic ignored "-Wunused-variable"
#include "font8x8_basic.h"
#pragma GCC diagnostic pop

// =============================================================================
// display.c — framebuffer + text rendering over esp_lcd SSD1306 panel
// =============================================================================
#include "display.h"
#include "font8x8_basic.h"
#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// SSD1306 page layout: byte at fb[x + page*128] controls 8 vertical pixels.
// Bit 0 = topmost pixel of the page, bit 7 = bottommost.

static inline void _set_pixel(display_t *d, int x, int y, bool on)
{
    if (x < 0 || x >= DISP_W || y < 0 || y >= DISP_H) return;
    int page = y / 8;
    int bit  = y % 8;
    int idx  = x + page * DISP_W;
    if (on) d->fb[idx] |=  (1u << bit);
    else    d->fb[idx] &= ~(1u << bit);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void display_init(display_t *d, esp_lcd_panel_handle_t panel)
{
    d->panel = panel;
    display_clear(d, 0x00);
    display_flush(d);
}

void display_flush(display_t *d)
{
    // draw_bitmap(panel, x_start, y_start, x_end, y_end, color_data)
    // For SSD1306 the panel driver interprets color_data as the raw page buffer.
    esp_lcd_panel_draw_bitmap(d->panel, 0, 0, DISP_W, DISP_H, d->fb);
}

// ---------------------------------------------------------------------------
// Framebuffer primitives
// ---------------------------------------------------------------------------

void display_clear(display_t *d, uint8_t fill)
{
    memset(d->fb, fill, DISP_FB_BYTES);
}

void display_set_pixel(display_t *d, int x, int y, bool on)
{
    _set_pixel(d, x, y, on);
}

void display_hline(display_t *d, int x, int y, int w, bool on)
{
    for (int i = 0; i < w; i++) _set_pixel(d, x + i, y, on);
}

void display_vline(display_t *d, int x, int y, int h, bool on)
{
    for (int i = 0; i < h; i++) _set_pixel(d, x, y + i, on);
}

void display_rect(display_t *d, int x, int y, int w, int h, bool on)
{
    display_hline(d, x,         y,         w, on);
    display_hline(d, x,         y + h - 1, w, on);
    display_vline(d, x,         y,         h, on);
    display_vline(d, x + w - 1, y,         h, on);
}

void display_fill_rect(display_t *d, int x, int y, int w, int h, bool on)
{
    for (int row = y; row < y + h; row++)
        display_hline(d, x, row, w, on);
}

// ---------------------------------------------------------------------------
// Text rendering
// ---------------------------------------------------------------------------

void display_draw_char(display_t *d, int x, int y, char c, bool inv)
{
    uint8_t idx = (uint8_t)c;
    if (idx > 127) idx = '?';

    for (int col = 0; col < 8; col++) {
        // uint8_t col_data = font8x8_basic[idx][col];
        uint8_t col_data = font8x8_basic_tr[idx][col];   // was font8x8_basic
        for (int row = 0; row < 8; row++) {
            bool pixel = (col_data >> row) & 1;
            if (inv) pixel = !pixel;
            _set_pixel(d, x + col, y + row, pixel);
        }
    }
}

void display_draw_str(display_t *d, int x, int y, const char *s, bool inv)
{
    while (*s) {
        if (x + 8 > DISP_W) break;
        display_draw_char(d, x, y, *s, inv);
        x += 8;
        s++;
    }
}

void display_text(display_t *d, int col, int row, const char *s, bool inv)
{
    display_draw_str(d, col * 8, row * 8, s, inv);
}

void display_draw_str_2x(display_t *d, int x, int y, const char *s, bool inv)
{
    while (*s) {
        uint8_t idx = (uint8_t)*s;
        if (idx > 127) idx = '?';

        for (int col = 0; col < 8; col++) {
            // uint8_t col_data = font8x8_basic[idx][col];
            uint8_t col_data = font8x8_basic_tr[idx][col];   // was font8x8_basic
            for (int row = 0; row < 8; row++) {
                bool pixel = (col_data >> row) & 1;
                if (inv) pixel = !pixel;
                // Each pixel → 2x2 block
                _set_pixel(d, x + col * 2,     y + row * 2,     pixel);
                _set_pixel(d, x + col * 2 + 1, y + row * 2,     pixel);
                _set_pixel(d, x + col * 2,     y + row * 2 + 1, pixel);
                _set_pixel(d, x + col * 2 + 1, y + row * 2 + 1, pixel);
            }
        }
        x += 16;
        s++;
        if (x + 16 > DISP_W) break;
    }
}

// ---------------------------------------------------------------------------
// Higher-level UI helpers
// ---------------------------------------------------------------------------

void display_status_bar(display_t *d, const char *label, const char *value)
{
    // Row 0: filled white bar, black text
    display_fill_rect(d, 0, 0, DISP_W, 8, true);

    // Label left-aligned
    display_draw_str(d, 0, 0, label, true);  // inv=true → black on white

    // Value right-aligned (up to 6 chars fits in 128px)
    int vlen = 0;
    const char *p = value;
    while (*p++) vlen++;
    int vx = DISP_W - (vlen * 8);
    if (vx < 0) vx = 0;
    display_draw_str(d, vx, 0, value, true);
}

void display_data_row(display_t *d, int row, const char *label, const char *value)
{
    // Clear the row first
    display_fill_rect(d, 0, row * 8, DISP_W, 8, false);

    display_draw_str(d, 0, row * 8, label, false);

    // Right-align value
    int vlen = 0;
    const char *p = value;
    while (*p++) vlen++;
    int vx = DISP_W - (vlen * 8);
    if (vx < 0) vx = 0;
    display_draw_str(d, vx, row * 8, value, false);
}

void display_bar(display_t *d, int row, int percent)
{
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    int y = row * 8 + 1;
    int bar_w = DISP_W - 2;  // 126 px wide, 1px border each side
    int filled = (bar_w * percent) / 100;

    // Outer border
    display_rect(d, 0, y, DISP_W, 6, true);
    // Filled portion
    if (filled > 0)
        display_fill_rect(d, 1, y + 1, filled, 4, true);
    // Empty portion
    if (filled < bar_w)
        display_fill_rect(d, 1 + filled, y + 1, bar_w - filled, 4, false);
}

void display_invert_row(display_t *d, int row)
{
    int start = row * DISP_W;
    for (int i = 0; i < DISP_W; i++) {
        d->fb[start + i] ^= 0xFF;
    }
}
