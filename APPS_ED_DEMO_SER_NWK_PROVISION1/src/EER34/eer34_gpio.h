/*
 * Electronica Elemon SA
 *
 * gpio.h
 *
 * Created: 03/12/2019 
 *  Author: mciancio
 */ 
 
#ifndef GPIO_H_
#define GPIO_H_

#include <port.h>

// Macros

// Data types

typedef enum {
	INPUT = 0,
	OUTPUT,
	INPUT_PULLUP,
	INPUT_PULLDOWN
} pinModetype_t;

// Public variables


// Prototypes

void EER34_Gpio_pinMode      ( const uint8_t gpio_pin , pinModetype_t mode );
void EER34_Gpio_digitalWrite ( const uint8_t gpio_pin , const bool value );
bool EER34_Gpio_digitalRead  ( const uint8_t gpio_pin );

#endif /* GPIO_H_ */