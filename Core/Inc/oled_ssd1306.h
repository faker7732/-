#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H
void OLED_Init(void);
void OLED_ShowStatus(float target_rpm, float actual_rpm, float pwm_percent);
#endif
