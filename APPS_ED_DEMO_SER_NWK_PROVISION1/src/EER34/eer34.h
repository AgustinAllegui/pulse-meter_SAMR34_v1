/*
 * Electronica Elemon SA
 *
 * eer34.h
 *
 * Created: 22/10/2019 11:52:42 a. m.
 *  Author: gmont_000
 */ 
#ifndef EER34_H_
#define EER34_H_

#include "asf.h"
#include "lorawan.h"

// Macros

#define EER34_TXBUFFER_SIZE		128		/**< tamaño del buffer de transmision */
#define EER34_DEF_TX_TIMEOUT	20000	/**< timeout default de tramsision en ms */
#define EER34_MAX_NA_CHANNELS		72
#define EER34_MAX_SUBBAND_CHANNELS	8

// Tipos de dato

typedef enum {
	EER34_STATUS_UNKNOWN = 0,
	EER34_STATUS_JOIN_SUCCSESS,
	EER34_STATUS_JOIN_ERROR,
	EER34_STATUS_TX_SUCCSESS,
	EER34_STATUS_TX_ACK,
	EER34_STATUS_TX_TIMEOUT,
	EER34_STATUS_RX_ERROR,
} EER34_status_t;

typedef enum {
	EER34_TXMODE_UNCONF = 0,
	EER34_TXMODE_CONF,
} EER34_txMode_t;

typedef enum {
	EER34_ADR_OFF = 0,
	EER34_ADR_ON,
} EER34_adrMode_t;

// Variables publicas

extern int EER34_txTimeout;					///< timeout del comando de transmision en ms
extern StackRetStatus_t EER34_txStatus;		///< status del stack LoRaWan de la ultma transmision

// Prototipos

void EES34_reset(void);
void EER34_init(void);
void EER34_tickStart(int t);
void EER34_tickStop(void);
int EER34_setDevEui(uint8_t *devEui);
int EER34_setAppEui(uint8_t *appEui);
int EER34_setAppKey(uint8_t *appKey);
int EER34_setDevAddr(uint32_t *devAddr);
int EER34_setAppSKey(uint8_t *appSKey);
int EER34_setNwkSKey(uint8_t *nwkSKey);
int EER34_setDeviceClass(EdClass_t class);
int EER34_setBand(IsmBand_t band, int subBand);
int EER34_setAdr(EER34_adrMode_t adr);
int EER34_joinOTAA(void);
int EER34_joinABP(void);
int EER34_tx(EER34_txMode_t mode, int port, uint8_t *data, int len);
int EER34_sleep(uint32_t time);
void EER34_statusCallback(EER34_status_t sts, StackRetStatus_t LoraSts);
void EER34_rxDataCallback(int port, uint8_t *data, int len);
void EER34_tickCallback(void);
void EES34_appInit(void);
void EES34_appTask(void);
void EES34_appResetCallback(unsigned int rcause);
void EES34_enterLowPower(void);
void EES34_exitLowPower(void);
int EER34_getchar(void);
void EER34_getLineInit(char *buffer, int size);
int EER34_getLine(void);

#endif /* EER34_H_ */
