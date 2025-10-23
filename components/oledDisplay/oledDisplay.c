#include <stdio.h>
#include <string.h>
#include "oledDisplay.h"
#include "ssd1306.h"
#include "font6x8.h"


const uint8_t *transposeBitMap(uint8_t c){
    char *letter = font6x8_ascii[c];
    uint8_t *out = malloc(7);
    for(uint8_t c = 1; c<8; c++){
        for(uint8_t r = 1; r<6; r++){
            if(letter[r] & (1 << (7-c)))
                out[7-c] |= 1 << (8-r);
            else out[7-c] &= ~(1 << (8-r));
        }
    }
    return out;

}

void ssd1306_printChar(ssd1306_handle_t display, uint8_t chXpos, uint8_t chYpos,
                         uint8_t c)
{
    ssd1306_draw_bitmap(display, chXpos, chYpos, transposeBitMap(c), 5, 7);
}

uint8_t ssd1306_printLine(ssd1306_handle_t display, uint8_t *chXpos, uint8_t *chYpos,
                         const uint8_t *pchString)
{
    if(*chXpos > SSD1306_WIDTH - 6 || *chYpos > SSD1306_HEIGHT - 8){
        return -1;
    }
    uint8_t charPrinted = 0;
    while (*(pchString + charPrinted) != '\0') {
        if(*chXpos > SSD1306_WIDTH - 6){
            return charPrinted;
        }
        ssd1306_printChar(display,*chXpos,*chYpos,*(pchString + charPrinted));
        *chXpos += 6;
        charPrinted++;
    }
    return charPrinted;
}

uint8_t ssd1306_printLineN(ssd1306_handle_t display, uint8_t *chXpos, uint8_t *chYpos,
                         const uint8_t *pchString)
{
    uint8_t out = ssd1306_printLine(display,chXpos,chYpos,pchString);
    *chXpos = 0;
    *chYpos += 8;
    return out;
}