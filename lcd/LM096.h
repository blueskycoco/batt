#ifndef __LM096_H
#define __LM096_H

#ifdef __cplusplus
 extern "C" {
#endif
void ssd1306_init(void);
void drawstring(uint8_t x, uint8_t line, char *c);
void display(void);
#ifdef __cplusplus
}
#endif

#endif
