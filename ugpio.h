/*
 *
 * File : ugpio.h
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

#ifndef __UGPIO_H__
#define __UGPIO_H__

#include "c_types.h"

typedef void (*handlerPtr)(int);

typedef enum {
    UGPIO_OUTPUT=0,
    UGPIO_INPUT=1
} UGPIO_PinMode;

typedef enum {
    UGPIO_NOPULL=0,
    UGPIO_PULLUP=1
} UGPIO_PullMode;

typedef struct {
    uint32_t ugpio_mux_name;
    uint8_t ugpio_func;
    uint8_t ugpio_available;
    UGPIO_PinMode ugpio_pin_mode;
    UGPIO_PullMode ugpio_pull_mode;
    handlerPtr funcPtr;
} ugpio_reg;

uint8_t ugpio_getAvailable(uint8_t ugpio_pin);

bool ugpio_setUnavailable(uint8_t ugpio_pin);

bool ugpio_inputMode(uint8_t ugpio_pin, UGPIO_PullMode pullMode, void (*interruptHandlerFunc)(void *args));

bool ugpio_outputMode(uint8_t ugpio_pin, UGPIO_PullMode pullMode);

bool ugpio_set(uint8_t ugpio_pin, uint8_t value);

uint8_t ugpio_get(uint8_t ugpio_pin);

bool ugpio_setFree(uint8_t ugpio_pin);

void ugpioInit(void);

#endif
