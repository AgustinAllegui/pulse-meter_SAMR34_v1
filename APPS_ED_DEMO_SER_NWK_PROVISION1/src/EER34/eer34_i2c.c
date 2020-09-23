/*
 * Electronica Elemon SA
 *
 * EER34_i2c.c
 *
 * Created: 03/12/2019 
 *  Author: mciancio
 */ 

#include "EER34_i2c.h"

void EER34_I2C_Read( uint8_t Address, uint8_t * read_buffer, uint8_t size )
{
	uint32_t timeout = 0;	

	master_packet.address         = Address;
	master_packet.data_length     = size;
	master_packet.data            = read_buffer;
	master_packet.ten_bit_address = false;
	master_packet.high_speed      = false;
	master_packet.hs_master_code  = 0x0;	
	
	while (i2c_master_read_packet_wait( &i2c_master_instance , &master_packet ) !=	STATUS_OK) {
		if (timeout++ == TIMEOUT) {
			return;
		}
	}
}

void EER34_I2C_Write( uint8_t Address, uint8_t * write_buffer, uint8_t size )
{
	uint32_t timeout = 0;

	master_packet.address         = Address;
	master_packet.data_length     = size;
	master_packet.data            = write_buffer;
	master_packet.ten_bit_address = false;
	master_packet.high_speed      = false;
	master_packet.hs_master_code  = 0x0;
		
	while ( i2c_master_write_packet_wait_no_stop ( &i2c_master_instance , &master_packet ) != STATUS_OK ) {
		/* Increment timeout counter and check if timed out. */
		if ( timeout++ == TIMEOUT ) {
			return;
		}
	}
}

void EER34_I2C_begin( void )
{
	/* Initialize config structure */
	i2c_master_get_config_defaults ( &config_i2c );

	/* Change pins */
	config_i2c.pinmux_pad0  = EDBG_I2C_SERCOM_PINMUX_PAD0;
	config_i2c.pinmux_pad1  = EDBG_I2C_SERCOM_PINMUX_PAD1;

	/* Initialize and enable device with config */
	i2c_master_init   ( &i2c_master_instance , EDBG_I2C_MODULE , &config_i2c );

	i2c_master_enable ( &i2c_master_instance );
}
