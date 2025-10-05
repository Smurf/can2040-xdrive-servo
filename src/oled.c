#include "pico.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "pico-oled-driver/picoOled.h"

#define PICO_LED 25
#define DEBUG_TRIG 22

#define UINP0 10
#define UINP1 7
#define UINP2 6


t_OledParams g_sh1106_oled;

// Initialize the oled structure with the information
//  required to define the display and then configure it
void configOled(t_OledParams *oled){

    oled->i2c         = i2c1;
    oled->SDA_PIN     = 14;
    oled->SCL_PIN     = 15;

    oled->ctlrType    = CTRL_SH1106;
    oled->i2c_address = 0x3C;
    oled->height      = H_64;
    oled->width       = W_132;

// this will configure the OLED module and then
//  clear the screen.
    oledI2cConfig(oled);

}

void initOled(t_OledParams *oled, bool tty_mode){
    configOled(oled);

    if( tty_mode == true ){

        oledSetTTYMode(oled, true);
    }
}