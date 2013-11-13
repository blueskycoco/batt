#ifndef __LM096_H
#define __LM096_H

#ifdef __cplusplus
 extern "C" {
#endif
void ssd1306_init(void);
//void drawstring(uint8_t x, uint8_t line, char *c);
void display(void);
void clear(void);
void draw(uint8_t bat1,uint8_t bat2,char *c);
#ifdef __cplusplus
}
#endif

#endif
