/*
    @TODOLIST
    -General debugging (first sight seams to work great)
*/

#ifndef __UGPIO_H__
#define __UGPIO_H__

#include "c_types.h"

// Define ugpio interrupt callback func
typedef void (*handlerPtr)(void *args);

typedef enum {
    UGPIO_OUTPUT=0,
    UGPIO_INPUT=1
} UGPIO_PinMode;

typedef enum {
    UGPIO_NOPULL=0,
    UGPIO_PULLUP=1
} UGPIO_PullMode;

// Define mapping from pin to function mode
typedef struct {
    uint32_t ugpio_mux_name;
    uint8_t ugpio_func;
    uint8_t ugpio_available;
    UGPIO_PinMode ugpio_pin_mode;
    UGPIO_PullMode ugpio_pull_mode;
    handlerPtr funcPtr;
} ugpio_reg;

// MY VERSION OF GPIO MANAGMENT

uint8_t ugpio_getAvailable(uint8_t ugpio_pin);//DONE

bool ugpio_setUnavailable(uint8_t ugpio_pin);//DONE
// With or without callback, just in case for gpio 16 (doesn't make much sense, but for completeness...)
bool ugpio_inputMode(uint8_t ugpio_pin, UGPIO_PullMode pullMode, void (*interruptHandlerFunc)(void *args));

bool ugpio_outputMode(uint8_t ugpio_pin, UGPIO_PullMode pullMode);

bool ugpio_set(uint8_t ugpio_pin, uint8_t value); // set output value

uint8_t ugpio_get(uint8_t ugpio_pin); // get input value

bool ugpio_setFree(uint8_t ugpio_pin); // call it before switch from input or output or just to free the gpio from list and interrupt
void ugpioInit(void);

#endif
