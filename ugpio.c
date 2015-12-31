/*
 *
 * File : ugpio.c
 *
 * Copyright (C) 2015-2016 Katast
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 3 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <esp8266.h>
#include "gpio.h"
#include "osapi.h"
#include "ets_sys.h"
#include "ugpio.h"

#ifdef UGPIO_DBG
#define DBG_UGPIO(format, ...) os_printf(format, ## __VA_ARGS__)
#else
#define DBG_UGPIO(format, ...) do { } while(0)
#endif

static ugpio_reg ugpio_map[] =
{
    { PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0, 1, 0, 0, NULL },	//gpio0
    { PERIPHS_IO_MUX_U0TXD_U, FUNC_GPIO1, 1, 0, 0, NULL },	//gpio1 (uart)
    { PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2, 1, 0, 0, NULL },	//gpio2 (dangerous pin, because it send data at startup)
    { PERIPHS_IO_MUX_U0RXD_U, FUNC_GPIO3, 1, 0, 0, NULL },	//gpio3 (uart)
    { PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4, 1, 0, 0, NULL },	//gpio4
    { PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5, 1, 0, 0, NULL },	//gpio5
    { 0, 0, -2, 0, 0, NULL },
    { 0, 0, -2, 0, 0, NULL },
    { 0, 0, -2, 0, 0, NULL },
    { PERIPHS_IO_MUX_SD_DATA2_U, FUNC_GPIO9, -2, 0, 0, NULL }, //gpio9  //Both useless in QIO
    { PERIPHS_IO_MUX_SD_DATA3_U, FUNC_GPIO10, -2, 0, 0, NULL }, //gpio10
    { 0, 0, -2, 0, 0, NULL },
    { PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12, 1, 0, 0, NULL },	//gpio12
    { PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13, 1, 0, 0, NULL },	//gpio13
    { PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14, 1, 0, 0, NULL },	//gpio14
    { PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15, 1, 0, 0, NULL },	//gpio15 (should be tied to ground)
    { 0, 0, 1, 0, 0, NULL },//@TODO gpio16 is different
};

static uint8_t intPinNum;

uint8_t ICACHE_FLASH_ATTR
interruptPinNumber(uint32_t x){
    uint8_t count;
    for (count=0; x != 0; x>>=1){
        if(x & 0x01){
            return count;
        }
        count++;
    }
    return 0xFF;
}
 
uint8_t ICACHE_FLASH_ATTR
ugpio_getAvailable(uint8_t ugpio_pin){
    return ugpio_map[ugpio_pin].ugpio_available;
}

bool ugpio_setUnavailable(uint8_t ugpio_pin){
    if(ugpio_getAvailable(ugpio_pin)){
        ugpio_map[ugpio_pin].ugpio_available = -1;
        return true;
    }else{
        return false;
    }
}

static void ICACHE_FLASH_ATTR
ugpio16_outputMode(void) {
  WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
      (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbcUL) | 0x1UL); // mux configuration for XPD_DCDC to output rtc_gpio0

  WRITE_PERI_REG(RTC_GPIO_CONF,
      (READ_PERI_REG(RTC_GPIO_CONF) & 0xfffffffeUL) | 0x0UL); //mux configuration for out enable

  WRITE_PERI_REG(RTC_GPIO_ENABLE,
      (READ_PERI_REG(RTC_GPIO_ENABLE) & 0xfffffffeUL) | 0x1UL); //out enable
}

static void ICACHE_FLASH_ATTR
ugpio16_inputMode(void) {
  WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
      (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbcUL) | 0x1UL); // mux configuration for XPD_DCDC and rtc_gpio0 connection

  WRITE_PERI_REG(RTC_GPIO_CONF,
      (READ_PERI_REG(RTC_GPIO_CONF) & 0xfffffffeUL) | 0x0UL); //mux configuration for out enable

  WRITE_PERI_REG(RTC_GPIO_ENABLE,
      READ_PERI_REG(RTC_GPIO_ENABLE) & 0xfffffffeUL);  //out disable
}

bool ICACHE_FLASH_ATTR
ugpio_inputMode(uint8_t ugpio_pin, UGPIO_PullMode pullMode, void (*interruptHandlerFunc)(void *args)){
    if (ugpio_getAvailable(ugpio_pin)){
        if (ugpio_pin == 16){
            ugpio16_inputMode();
            ugpio_map[ugpio_pin].ugpio_available = 0;
            ugpio_map[ugpio_pin].ugpio_pin_mode = UGPIO_INPUT;
            return true;
        }else{
            // Set pin function
            PIN_FUNC_SELECT(ugpio_map[ugpio_pin].ugpio_mux_name, ugpio_map[ugpio_pin].ugpio_func);
            
            if (pullMode){
                PIN_PULLUP_EN(ugpio_map[ugpio_pin].ugpio_mux_name);
                ugpio_map[ugpio_pin].ugpio_pull_mode = UGPIO_PULLUP;
            }else{
                PIN_PULLUP_DIS(ugpio_map[ugpio_pin].ugpio_mux_name);
                ugpio_map[ugpio_pin].ugpio_pull_mode = UGPIO_NOPULL;
            }
            // Input mode
            GPIO_DIS_OUTPUT(ugpio_pin);
            ugpio_map[ugpio_pin].ugpio_pin_mode = UGPIO_INPUT;
            
            ETS_GPIO_INTR_DISABLE();
     
            gpio_register_set(GPIO_PIN_ADDR(ugpio_pin), GPIO_PIN_INT_TYPE_SET(GPIO_PIN_INTR_DISABLE)
                                | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_DISABLE)
                                | GPIO_PIN_SOURCE_SET(GPIO_AS_PIN_SOURCE));
            //clear ugpio_pin status
            GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(ugpio_pin));
            
            if (interruptHandlerFunc != NULL){
                gpio_pin_intr_state_set(GPIO_ID_PIN(ugpio_pin), GPIO_PIN_INTR_ANYEDGE);
                ugpio_map[ugpio_pin].funcPtr = interruptHandlerFunc;
            }
            ETS_GPIO_INTR_ENABLE();
            ugpio_map[ugpio_pin].ugpio_available = 0;
            //DBG_UGPIO("UGPIO input mode fatto\n");
            return true;
        }
    }else{
        return false;
    }
}

bool ICACHE_FLASH_ATTR
ugpio_outputMode(uint8_t ugpio_pin, UGPIO_PullMode pullMode){
    if(ugpio_getAvailable(ugpio_pin)){
        if (ugpio_pin == 16){
            ugpio16_outputMode();
            ugpio_map[ugpio_pin].ugpio_pin_mode = UGPIO_OUTPUT;
            ugpio_map[ugpio_pin].ugpio_available = 0;
        }else{
            // Set pin function
            PIN_FUNC_SELECT(ugpio_map[ugpio_pin].ugpio_mux_name, ugpio_map[ugpio_pin].ugpio_func);
            gpio_output_set(0, 0, BIT(GPIO_ID_PIN(ugpio_pin)),0);
            ugpio_map[ugpio_pin].ugpio_pin_mode = UGPIO_OUTPUT;
            ugpio_map[ugpio_pin].ugpio_available = 0;
            
            if (pullMode){
                PIN_PULLUP_EN(ugpio_map[ugpio_pin].ugpio_mux_name);
                ugpio_map[ugpio_pin].ugpio_pull_mode = UGPIO_PULLUP;
            }else{
                PIN_PULLUP_DIS(ugpio_map[ugpio_pin].ugpio_mux_name);
                ugpio_map[ugpio_pin].ugpio_pull_mode = UGPIO_NOPULL;
            }
        }
        return true;
    }else{
        return false;
    }
}

bool ICACHE_FLASH_ATTR
ugpio_set(uint8_t ugpio_pin, uint8_t value){
    if(ugpio_getAvailable(ugpio_pin) == 0){
        if(ugpio_map[ugpio_pin].ugpio_pin_mode == UGPIO_OUTPUT){
            if (ugpio_pin == 16){
                WRITE_PERI_REG(RTC_GPIO_OUT,
                               (READ_PERI_REG(RTC_GPIO_OUT) & 0xfffffffeUL) | (0x1UL & value));
            }else{
                if (value&1){
                    WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR, READ_PERI_REG(PERIPHS_GPIO_BASEADDR) | BIT(ugpio_pin));
                } else {
                    WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR, READ_PERI_REG(PERIPHS_GPIO_BASEADDR) & ~BIT(ugpio_pin));
                }
            }
        }
        return true;
    }else{
        return false;
    }
}

uint8_t ICACHE_FLASH_ATTR
ugpio_get(uint8_t ugpio_pin){
    if(ugpio_getAvailable(ugpio_pin) == 0){
        if(ugpio_map[ugpio_pin].ugpio_pin_mode == UGPIO_INPUT){
            if (ugpio_pin == 16){
                return (READ_PERI_REG(RTC_GPIO_IN_DATA) & 1UL);
            }else{
                return GPIO_INPUT_GET(ugpio_pin);
            }
        }else
            return -2;
    }else
        return -1;
}

bool ICACHE_FLASH_ATTR
ugpio_setFree(uint8_t ugpio_pin){
    if (ugpio_getAvailable(ugpio_pin) == 0){
        if(ugpio_pin != 16){
            gpio_pin_intr_state_set(GPIO_ID_PIN(ugpio_pin), GPIO_PIN_INTR_DISABLE);
        }
        ugpio_map[ugpio_pin].funcPtr = NULL;
        ugpio_map[ugpio_pin].ugpio_available = 1;
        return true;
    }
    return false;
}

void ICACHE_FLASH_ATTR
ugpioMainInterrupt(uint8_t intPinNum){
    uint32_t ugpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    int ugpio_id = interruptPinNumber(ugpio_status);
    if (ugpio_id != 0xFF){
        // Disable interrupt on this pin
        gpio_pin_intr_state_set(GPIO_ID_PIN(ugpio_id), GPIO_PIN_INTR_DISABLE);
        // Callback to appropriate function
        ugpio_map[ugpio_id].funcPtr((int*)ugpio_id);
        // Clear interrupts
        GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, ugpio_status);
        // Reactivate interrupts for GPIO0
        gpio_pin_intr_state_set(GPIO_ID_PIN(ugpio_id), GPIO_PIN_INTR_ANYEDGE);
    }else{
        GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, ugpio_status);
    }
}

void ICACHE_FLASH_ATTR
ugpioInit(void){
    ETS_GPIO_INTR_ATTACH(ugpioMainInterrupt, &intPinNum);
    DBG_UGPIO("UGPIO started with success\n");
}