/*
 * Electrónica Elemon SA
 *
 * gpio.h
 *
 * Created: 03/12/2019 
 *  Author: mciancio
 */ 

#include "EER34_gpio.h"

/** 
 *	@brief	
 *	@param	
 *	@param	
 *	@return	
 */

void EER34_Gpio_pinMode (  const uint8_t gpio_pin , pinModetype_t mode )
{
	struct port_config config_port_pin;
	
	port_get_config_defaults(&config_port_pin);
	
	config_port_pin.direction = PORT_PIN_DIR_INPUT ;
	config_port_pin.input_pull = PORT_PIN_PULL_UP ;
	
	 if ( mode == OUTPUT )
	 {
		 config_port_pin.direction = PORT_PIN_DIR_OUTPUT ;
		 port_pin_set_config ( gpio_pin , &config_port_pin ) ;
	 }
	 else if ( mode == INPUT  )
	{
		config_port_pin.direction = PORT_PIN_DIR_INPUT ;
		config_port_pin.input_pull = PORT_PIN_PULL_NONE ;		
		port_pin_set_config ( gpio_pin , &config_port_pin ) ;	
	}
	else if ( mode == INPUT_PULLUP )
	{
		config_port_pin.direction = PORT_PIN_DIR_INPUT ;
		config_port_pin.input_pull = PORT_PIN_PULL_UP ;
		
		port_pin_set_config ( gpio_pin , &config_port_pin ) ;
	}
	else if ( mode == INPUT_PULLDOWN )
	{
		config_port_pin.direction = PORT_PIN_DIR_INPUT ;
		config_port_pin.input_pull = PORT_PIN_PULL_DOWN ;
	
		port_pin_set_config ( gpio_pin , &config_port_pin ) ;
	}
}

/** 
 *	@brief	
 *	@param	
 *	@param	
 *	@return	
 */

void EER34_Gpio_digitalWrite (  const uint8_t gpio_pin , const bool value )
{
	port_pin_set_output_level ( gpio_pin , value );
}

/** 
 *	@brief	
 *	@param	
 *	@param	
 *	@return	
 */

bool EER34_Gpio_digitalRead (  const uint8_t gpio_pin )
{
	if ( port_pin_get_input_level ( gpio_pin ) == false  ) return false;
		else return true;
}