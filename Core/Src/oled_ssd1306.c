#include "main.h"
#include "oled_ssd1306.h"
#include <stdio.h>
#include <string.h>
extern I2C_HandleTypeDef hi2c1;

#define OLED_ADDR (0x3CU << 1)
static void command_list(const uint8_t *cmd, uint16_t n) {
  uint8_t buf[32]; if(n>31U)n=31U; buf[0]=0x00; memcpy(&buf[1],cmd,n);
  HAL_I2C_Master_Transmit(&hi2c1,OLED_ADDR,buf,n+1U,50U);
}
static void command(uint8_t c){ command_list(&c,1U); }
static void set_pos(uint8_t page,uint8_t col){uint8_t c[3]={(uint8_t)(0xB0U|page),(uint8_t)(col&0x0FU),(uint8_t)(0x10U|(col>>4))};command_list(c,3U);}

/* 5x7 ASCII glyphs for status labels, digits and punctuation. */
static void glyph(char c, uint8_t g[5]) {
 static const uint8_t digits[10][5]={{0x3e,0x51,0x49,0x45,0x3e},{0,0x42,0x7f,0x40,0},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},{0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3c,0x4a,0x49,0x49,0x30},{1,0x71,9,5,3},{0x36,0x49,0x49,0x49,0x36},{6,0x49,0x49,0x29,0x1e}};
 const uint8_t *p=0;
 if(c>='0'&&c<='9'){memcpy(g,digits[c-'0'],5);return;}
 switch(c){
 case 'T': p=(const uint8_t[]){1,1,0x7f,1,1}; break;
 case 'A': p=(const uint8_t[]){0x7e,0x11,0x11,0x11,0x7e}; break;
 case 'R': p=(const uint8_t[]){0x7f,9,0x19,0x29,0x46}; break;
 case 'G': p=(const uint8_t[]){0x3e,0x41,0x49,0x49,0x7a}; break;
 case 'E': p=(const uint8_t[]){0x7f,0x49,0x49,0x49,0x41}; break;
 case 'L': p=(const uint8_t[]){0x7f,0x40,0x40,0x40,0x40}; break;
 case 'C': p=(const uint8_t[]){0x3e,0x41,0x41,0x41,0x22}; break;
 case 'P': p=(const uint8_t[]){0x7f,9,9,9,6}; break;
 case 'U': p=(const uint8_t[]){0x3f,0x40,0x40,0x40,0x3f}; break;
 case 'W': p=(const uint8_t[]){0x7f,0x20,0x18,0x20,0x7f}; break;
 case 'M': p=(const uint8_t[]){0x7f,2,0x0c,2,0x7f}; break;
 case ':': p=(const uint8_t[]){0,0x36,0x36,0,0}; break;
 case '.': p=(const uint8_t[]){0,0x60,0x60,0,0}; break;
 case '%': p=(const uint8_t[]){0x63,0x13,8,0x64,0x63}; break;
 default: memset(g,0,5); return;
 }
 memcpy(g,p,5);
}
static uint8_t draw_text(uint8_t page,uint8_t col,const char *s){uint8_t g[5],i,buf[129],n=0;buf[0]=0x40;while(*s&&col<128U){glyph(*s++,g);for(i=0;i<5U&&col<128U;i++,col++)buf[++n]=g[i];if(col<128U){buf[++n]=0;col++;}}set_pos(page,(uint8_t)(col-n));HAL_I2C_Master_Transmit(&hi2c1,OLED_ADDR,buf,(uint16_t)(n+1U),50U);return n;}
void OLED_Init(void){static const uint8_t init[]={0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0,0x40,0x8D,0x14,0x20,0x02,0xA1,0xC8,0xDA,0x12,0x81,0x7F,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,0xAF};HAL_Delay(100);command_list(init,sizeof(init));}
static void clear_page(uint8_t page){uint8_t b[129];memset(&b[1],0,128);b[0]=0x40;set_pos(page,0);HAL_I2C_Master_Transmit(&hi2c1,OLED_ADDR,b,129,50U);}
void OLED_ShowStatus(float target,float actual,float pwm){char actual_text[24],target_text[16],pwm_text[16];uint8_t width;snprintf(actual_text,sizeof(actual_text),"ACTUAL: %.1f RPM",actual);snprintf(target_text,sizeof(target_text),"T:%3.0f",target);snprintf(pwm_text,sizeof(pwm_text),"PWM:%3.0f%%",pwm);width=(uint8_t)(strlen(actual_text)*6U);clear_page(0);clear_page(3);draw_text(0,(uint8_t)((128U-width)/2U),actual_text);draw_text(3,0,target_text);draw_text(3,80,pwm_text);}
