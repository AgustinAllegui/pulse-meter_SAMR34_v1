/*
 * Electronica Elemon SA
 *
 * spi.h
 *
 * Created: 03/12/2019 
 *  Author: mciancio
 */ 
 
#ifndef SPI_H_
#define SPI_H_

#include <port.h>
#include <sercom.h>

#include <spi.h>

#include "samr34_xplained_pro.h"

// Macros
#define SLAVE_SELECT_PIN EXT1_PIN_SPI_SS_0
// Tipos de dato

// Variables publicas

struct spi_module spi_master_instance;static struct spi_slave_inst slave;

struct spi_config config_spi_master;
struct spi_slave_inst_config slave_dev_config;

// Prototipes

void EER34_configureSpiMaster  ( uint32_t baudrate , enum spi_data_order dataOrder , enum spi_transfer_mode ModeSpi ) ;
void EER34_spiStartTransaction ( void ) ;void EER34_spiEndTransaction   ( void ) ;

uint8_t EER34_spiTransferVal        ( const uint8_t val ) ;
void    EER34_spiTransferBuffer     ( uint8_t * buffer , const uint8_t size );

#endif /* SPI_H_ */