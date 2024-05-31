#pragma once
#include <gfx.hpp>
#include <uix.hpp>
#ifndef USE_SINGLE_BUFFER
constexpr const size_t lcd_transfer_buffer_size = 32*1024;
#else
constexpr const size_t lcd_transfer_buffer_size = 64*1024;
#endif
using screen_t = uix::screen<gfx::rgb_pixel<16>>;
screen_t *lcd_active_screen();
void lcd_active_screen(screen_t* value);
void lcd_panel_init();
void lcd_on_flush(const gfx::rect16 &bounds, const void *bmp, void *state);
