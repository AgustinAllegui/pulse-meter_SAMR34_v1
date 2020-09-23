/*
 * Electronica Elemon SA
 *
 * EER34_i2c.h
 *
 * Created: 03/12/2019 
 *  Author: mciancio
 */ 
 
#ifndef I2C_H_
#define I2C_H_

#include <i2c_master.h>

// Macros
#define TIMEOUT						20
#define SLAVE_WAIT_TIMEOUT			10

#define EDBG_I2C_MODULE   					SERCOM1
#define EDBG_I2C_SERCOM_PINMUX_PAD0			PINMUX_PA16C_SERCOM1_PAD0
#define EDBG_I2C_SERCOM_PINMUX_PAD1			PINMUX_PA17C_SERCOM1_PAD1// Tipos de dato

// Variables publicas

/* Create and initialize config structure */
struct i2c_master_config config_i2c;
struct i2c_master_packet master_packet;
struct i2c_master_module i2c_master_instance;

// Prototipes

void EER34_I2C_begin ( void );
void EER34_I2C_Read  ( uint8_t Address , uint8_t * read_buffer  , uint8_t size );
void EER34_I2C_Write ( uint8_t Address , uint8_t * write_buffer , uint8_t size );

#endif /* I2C_H_ */