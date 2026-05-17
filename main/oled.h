#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool oled_init(void);
void oled_clear(void);
void oled_print(int col_px, int page, const char *text);
void oled_flush(void);

#ifdef __cplusplus
}
#endif
