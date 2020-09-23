/*
 * Electronica Elemon SA
 *
 * gpio.h
 *
 * Created: 03/12/2019 
 *  Author: mciancio
 */ 
 
#ifndef ADC_H_
#define ADC_H_

#include "adc.h"

// Macros

// Tipos de dato

// Variables publicas
struct adc_module adc_instance;

// Prototipes
void		EER34_Adc_startAdc		( void );
uint16_t	EER34_Adc_digitalRead	( void );

#endif /* ADC_H_ */