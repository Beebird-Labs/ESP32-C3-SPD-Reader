#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    bool oled_init(void);
    void oled_clear(void);
    void oled_print(int col_px, int page, const char *text);
    void oled_print_scaled(int col_px, int row_px, const char *text, int scale);
    void oled_flush(void);

#ifdef __cplusplus
}
#endif
