/*
 * EER34_nvm.h
 *
 * Created: 25/9/2020 09:33:50
 *  Author: Agustin
 */ 


#ifndef EER34_NVM_H_
#define EER34_NVM_H_

#include "eeprom.h"
#include "logMacros.h"

#define DEFAULT_PERIOD 30

#define PERIOD_EEPROM_PAGE 0
#define PULSE_COUNT_FIRST_EEPROM_PAGE 2
#define PULSE_COUNT_EEPROM_PAGES 10

static bool periodIsCorrupted = false;
static bool pulseCountIsCorrupted = false;


void configure_eeprom(void);
static void configure_bod(void);


uint32_t getSavedPulseCount(void);
void savePulseCount(uint32_t pulseCount);
void clearPulseCount(void);

uint32_t getSavedPeriod(void);
void savePeriod(const uint32_t period);



uint32_t readSinglePage(const uint8_t page);
void writeSinglePage(const uint8_t page, const uint32_t dataToBeStoraged);








#endif /* EER34_NVM_H_ */