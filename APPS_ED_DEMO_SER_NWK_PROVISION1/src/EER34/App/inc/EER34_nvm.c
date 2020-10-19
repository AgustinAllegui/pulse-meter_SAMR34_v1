/*
 * EER34_nvm.c
 *
 * Created: 25/9/2020 09:33:28
 *  Author: Agustin
 */ 

#include "EER34_nvm.h"
#include "logMacros.h"


/* Inicializa el servicio de emulacion de eeprom
 */
void configure_eeprom(void)
{
	/* Setup EEPROM emulator service */
    enum status_code error_code = eeprom_emulator_init();
    if (error_code == STATUS_ERR_NO_MEMORY) {
		logFatal("Fuses for EEPROM not setted");
        while (true) {
            /* No EEPROM section has been set in the device's fuses */
        }
    }
    else if (error_code != STATUS_OK) {
        /* Erase the emulated EEPROM memory (assume it is unformatted or
         * irrecoverably corrupt) */
		logError("Memory lost or corrupted");
		periodIsCorrupted = true;
		pulseCountIsCorrupted = true;
        eeprom_emulator_erase_memory();
        eeprom_emulator_init();
    }
	
	#if LOG_LEVEL >= DEBUG_LEVEL
	
	struct eeprom_emulator_parameters eeprom_parameters;
	eeprom_emulator_get_parameters(&eeprom_parameters);
	logDebug("Emulated pages in EEPROM: %u", eeprom_parameters.eeprom_number_of_pages);
	logDebug("Bytes per page: %u", eeprom_parameters.page_size);
	
	#endif
}

static void configure_bod(void)
{
	#if (SAMD || SAMR21)
	struct bod_config config_bod33;
	bod_get_config_defaults(&config_bod33);
	config_bod33.action = BOD_ACTION_INTERRUPT;
	/* BOD33 threshold level is about 3.2V */
	config_bod33.level = 48;
	bod_set_config(BOD_BOD33, &config_bod33);
	bod_enable(BOD_BOD33);
	SYSCTRL->INTENSET.reg = SYSCTRL_INTENCLR_BOD33DET;
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SYSCTRL);
	#endif
}


uint32_t getSavedPeriod(void)
{
	logTrace("Reading saved Period");
	if(periodIsCorrupted){
		logWarning("Period lost. Setting to %lu seg", DEFAULT_PERIOD);
		savePeriod(DEFAULT_PERIOD);
		return DEFAULT_PERIOD;
	}

	return readSinglePage(PERIOD_EEPROM_PAGE);
}


void savePeriod(const uint32_t period)
{
	logTrace("Saving period");
	writeSinglePage(PERIOD_EEPROM_PAGE, period);
	periodIsCorrupted = false;
}


uint32_t getSavedPulseCount(void)
{
	logTrace("Reading saved Pulse count");
	if(pulseCountIsCorrupted){
		logWarning("Pulse count lost. Setting to 0");
		savePulseCount(0);
		return 0;
	}
	
	// buscar la pagina con el mayor numero de pulsos contados
	uint32_t maxCount = 0;
	
	for(uint8_t i = 0; i<PULSE_COUNT_EEPROM_PAGES; i++){
		uint32_t currentCount = readSinglePage(PULSE_COUNT_FIRST_EEPROM_PAGE+i);
		if(currentCount > maxCount){
			maxCount = currentCount;
		}
	}
	
	return maxCount;
	
}


void savePulseCount(uint32_t pulseCount)
{
	logTrace("Saving pulse count");
	// buscar la pagina con el minimo conteo de pulsos
	uint32_t minCount = 0xFFFFFFFF;
	uint32_t maxCount = 0;
	uint8_t minCountIndex = 0;
	
	for(uint8_t i = 0; i<PULSE_COUNT_EEPROM_PAGES; i++){
		uint32_t currentCount = readSinglePage(PULSE_COUNT_FIRST_EEPROM_PAGE+i);
		if(currentCount < minCount){
			minCount = currentCount;
			minCountIndex = i;
		}
		if(currentCount > maxCount){
			maxCount = currentCount;
		}
	}
	
	if(maxCount == pulseCount){
		logDebug("Value already saved");
		return;
	}

	
	logDebug("Saving in page %u", minCountIndex);
	
	// escribir en esa pagina
	writeSinglePage(PULSE_COUNT_FIRST_EEPROM_PAGE+minCountIndex, pulseCount);
	
	
	pulseCountIsCorrupted = false;
}

void clearPulseCount(void)
{
	logInfo("Clearing saved pulse count");
	
	uint32_t periodBuffer = getSavedPeriod();
	eeprom_emulator_erase_memory();

	savePeriod(periodBuffer);
	
	
}

uint32_t readSinglePage(const uint8_t page)
{
	uint8_t page_data[EEPROM_PAGE_SIZE];
	eeprom_emulator_read_page(page, page_data);
	
	uint32_t storagedData = ((uint32_t)page_data[0] << 24) | ((uint32_t)page_data[1] << 16) | ((uint32_t)page_data[2] << 8) | (uint32_t)page_data[3];
	
	
	logDebug("Read value: %lu", storagedData == 0xFFFFFFFF ? 0 : storagedData);
	
	return storagedData == 0xFFFFFFFF ? 0 : storagedData;
}


void writeSinglePage(const uint8_t page, const uint32_t dataToBeStoraged)
{
	uint8_t page_data[EEPROM_PAGE_SIZE];
	
	page_data[0] = (dataToBeStoraged >> 24) & 0x00FF;
	page_data[1] = (dataToBeStoraged >> 16) & 0x00FF;
	page_data[2] = (dataToBeStoraged >> 8) & 0x00FF;
	page_data[3] = (dataToBeStoraged) & 0x00FF;
	
	eeprom_emulator_write_page(page, page_data);
	eeprom_emulator_commit_page_buffer();
}

