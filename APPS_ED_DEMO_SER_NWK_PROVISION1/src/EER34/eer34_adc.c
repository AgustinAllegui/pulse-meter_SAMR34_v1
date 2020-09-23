/*
 * Electronica Elemon SA
 *
 * adc.h
 *
 * Created: 03/12/2019 
 *  Author: mciancio
 */ 

#include "EER34_adc.h"

/** 
 *	@brief	
 *	@param	
 *	@param	
 *	@return	
 */

void EER34_Adc_startAdc ( void )
{
	struct adc_config config_adc;
	
	adc_get_config_defaults(&config_adc);
	
	adc_init   ( &adc_instance, ADC , &config_adc);
	adc_enable ( &adc_instance );
}

/** 
 *	@brief	
 *	@param	
 *	@param	
 *	@return	
 */

uint16_t EER34_Adc_digitalRead (void)
{
	adc_start_conversion ( &adc_instance );
	
	uint16_t result;
	do {
		
	} while ( adc_read ( &adc_instance, &result ) == STATUS_BUSY );
	
	return result;
}






 
