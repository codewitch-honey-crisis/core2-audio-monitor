#pragma once
#include <gfx.hpp>

void lcd_panel_init();
void lcd_on_flush(const gfx::rect16 &bounds, const void *bmp, void *state);
