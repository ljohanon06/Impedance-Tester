#include <stdio.h>
#include "ssd1306.h"
#include "font6x8.h"

const uint8_t *transposeBitMap(uint8_t c);

uint8_t ssd1306_printLine(ssd1306_handle_t display, uint8_t *chXpos, uint8_t *chYpos,
                         const uint8_t *pchString);

uint8_t ssd1306_printLineN(ssd1306_handle_t display, uint8_t *chXpos, uint8_t *chYpos,
                         const uint8_t *pchString);
