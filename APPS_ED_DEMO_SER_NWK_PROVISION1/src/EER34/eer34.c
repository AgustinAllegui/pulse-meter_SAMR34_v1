/*
 * Electronica Elemon SA
 *
 * eer34.c
 *
 * Created: 22/10/2019 11:50:58 a. m.
 *  Author: gmont_000
 */ 
#include "eer34.h"
#include <string.h>
#include "sw_timer.h"
#include "radio_driver_hal.h"
#include "sio2host.h"
#ifdef CONF_PMM_ENABLE
#include "pmm.h"
#include  "conf_pmm.h"
#include "sleep_timer.h"
#include "sleep.h"
#endif

// Variables publicas

int EER34_txTimeout;					///< timeout del comando de transmision en ms
StackRetStatus_t EER34_txStatus;		///< status del stack LoRaWan de la ultma transmision
StackRetStatus_t EER34_res;


// Variables privadas

static LorawanSendReq_t lorawanSendReq;
static uint8_t txBuffer[EER34_TXBUFFER_SIZE];
static uint8_t tickTimerId = 0xFF;
static uint8_t txTimerId = 0xFF;
static int tickInterval;
static struct {
	unsigned char *buffer;
	int size;
	int count;
} lineRdr;

// Prototipos privados

static void joinResponseCallback(StackRetStatus_t status);
static void appDataCallback(void *appHandle, appCbParams_t *appdata);
static void txTimerCb(void *data);
static void tickTimerCb(void *data);
SYSTEM_TaskStatus_t APP_TaskHandler(void);

/** 
 *	@brief	Setea el Device EUI
 *	@param	devEui	vector de 8 bytes de Device EUI (en el orden en que se lee)
 *	@return	1 si OK, 0 si fallo
 */
int EER34_setDevEui(uint8_t *devEui)
{
	EER34_res = LORAWAN_SetAttr(DEV_EUI, devEui);
	
	if (EER34_res == LORAWAN_SUCCESS)
		return 1;
		
	return 0;
}

/** 
 *	@brief	Setea el Application EUI
 *	@param	appEui	vector de 8 bytes de Application EUI (en el orden en que se lee)
 *	@return	1 si OK, 0 si fallo
 */
int EER34_setAppEui(uint8_t *appEui)
{
	EER34_res = LORAWAN_SetAttr(APP_EUI, appEui);
	
	if (EER34_res == LORAWAN_SUCCESS)
		return 1;
		
	return 0;
}

/** 
 *	@brief	Setea el Application Key
 *	@param	appKey	vector de 16 bytes de Application Key (en el orden en que se lee)
 *	@return	1 si OK, 0 si fallo
 */
int EER34_setAppKey(uint8_t *appKey)
{
	EER34_res = LORAWAN_SetAttr(APP_KEY, appKey);
	
	if (EER34_res == LORAWAN_SUCCESS)
		return 1;
		
	return 0;
}

/** 
 *	@brief	Setea el Device Address
 *	@param	devAddr		device Address
 *	@return	1 si OK, 0 si fallo
 */
int EER34_setDevAddr(uint32_t *devAddr)
{
	EER34_res = LORAWAN_SetAttr(DEV_ADDR, &devAddr);
	
	if (EER34_res == LORAWAN_SUCCESS)
		return 1;
		
	return 0;
}

/** 
 *	@brief	Setea el Application Session Key
 *	@param	devAddr	vector de 16 bytes de Application Session Key (en el orden en que se lee)
 *	@return	1 si OK, 0 si fallo
 */
int EER34_setAppSKey(uint8_t *appSKey)
{
	EER34_res = LORAWAN_SetAttr(APPS_KEY, &appSKey);
	
	if (EER34_res == LORAWAN_SUCCESS)
		return 1;
		
	return 0;
}

/** 
 *	@brief	Setea el modo de ADR
 *	@param	adr	modo de ADR: EER34_ADR_ON o EER34_ADR_OFF
 *	@return	1 si OK, 0 si fallo
 */
int EER34_setAdr(EER34_adrMode_t adr)
{
    bool adrValue = false;

	if (adr == EER34_ADR_ON)
		adrValue = true;
	
	EER34_res = LORAWAN_SetAttr(ADR, &adrValue);
	
	if (EER34_res == LORAWAN_SUCCESS)
		return 1;
	
	return 0;
}

/** 
 *	@brief	Setea el Network Session Key
 *	@param	nwkSKey	vector de 16 bytes de Application Session Key (en el orden en que se lee)
 *	@return	1 si OK, 0 si fallo
 */
int EER34_setNwkSKey(uint8_t *nwkSKey)
{
	EER34_res = LORAWAN_SetAttr(NWKS_KEY, &nwkSKey);
	
	if (EER34_res == LORAWAN_SUCCESS)
		return 1;
		
	return 0;
}

/** 
 *	@brief	Setea la clase de dispositivo
 *	@param	class	CLASS_A, CLASS_B, CLASS_C
 *	@return	1 si OK, 0 si fallo
 */
int EER34_setDeviceClass(EdClass_t class)
{
    EER34_res = LORAWAN_SetAttr(EDCLASS, &class);

	if (EER34_res == LORAWAN_SUCCESS)
		return 1;
	
	return 0;
}

/** 
 *	@brief	Setea la banda y la sub-banda (grupo de canales)
 *	@param	band		banda	(enum del stack LoraWan)
 *	@param	subBand		sub-banda	(1 a 8))
 *	@return	1 si OK, 0 si fallo
 */
int EER34_setBand(IsmBand_t band, int subBand)
{
    ChannelParameters_t chParams;
    uint8_t allowedMin125khzCh, allowedMax125khzCh,allowed500khzChannel;
    bool joinBackoffEnable = false;
	
	if (LORAWAN_Reset(band) != LORAWAN_SUCCESS)
		return 0;

	allowedMin125khzCh = (subBand-1)*EER34_MAX_SUBBAND_CHANNELS;
	allowedMax125khzCh = ((subBand-1)*EER34_MAX_SUBBAND_CHANNELS) + 7 ;
	allowed500khzChannel = subBand + 63;
	
    for (chParams.channelId = 0; chParams.channelId < EER34_MAX_NA_CHANNELS; chParams.channelId++)
    {
	    if((chParams.channelId >= allowedMin125khzCh) && (chParams.channelId <= allowedMax125khzCh))
	    {
		    chParams.channelAttr.status = true;
	    }
	    else if(chParams.channelId == allowed500khzChannel)
	    {
		    chParams.channelAttr.status = true;
	    }
	    else
	    {
		    chParams.channelAttr.status = false;
	    }

	    LORAWAN_SetAttr(CH_PARAM_STATUS, &chParams);
    }

    LORAWAN_SetAttr(JOIN_BACKOFF_ENABLE, &joinBackoffEnable);
	
	/// @todo	terminar de implementar 
	/// @see	processRunDemoApp(), mote_set_parameters(), processRunRestoreBand()

	return 1;
}

/** 
 *	@brief	Inicia el Join por Over-The-Air-Activation (OTAA)
 *	@return	1 si OK, 0 si fallo
 */
int EER34_joinOTAA(void)
{
	EER34_res = LORAWAN_Join(LORAWAN_OTAA);
	
	if (EER34_res == LORAWAN_SUCCESS)
		return 1;
		
	return 0;
}

/** 
 *	@brief	Inicia el Join por Activation-By-Personalization (ABP)
 *	@return	1 si OK, 0 si fallo
 */
int EER34_joinABP(void)
{
	EER34_res = LORAWAN_Join(LORAWAN_ABP);
	
	if (EER34_res == LORAWAN_SUCCESS)
		return 1;
		
	return 0;
}

/** 
 *	@brief	Inicializa el stack LoRaWan
 */
void EER34_init(void)
{
	// Defaults
	EER34_txTimeout = EER34_DEF_TX_TIMEOUT;
	
	// Crea los softwae timers
    SwTimerCreate(&txTimerId);
    SwTimerCreate(&tickTimerId);
	
	// Inicializa el stack LoRaWan
	LORAWAN_Init(appDataCallback, joinResponseCallback);
	
	// Inicializa la aplicacion
	EES34_appInit();
}

/** 
 *	@brief	Arranca el tick de aplicacion
 */
void EER34_tickStart(int t)
{
	tickInterval = t;
	
    SwTimerStart(tickTimerId, MS_TO_US(tickInterval),
		SW_TIMEOUT_RELATIVE, (void *)tickTimerCb, NULL);
}

/** 
 *	@brief	Detiene el tick de aplicacion
 */
void EER34_tickStop(void)
{
    SwTimerStop(tickTimerId);
}

/** 
 *	@brief	Transmite datos por LoRaWan
 *	@param	mode	modo de transmision (con o sin confirmacion)
 *	@param	port	numero de puerto
 *	@param	data	puntero a los bytes de datos a transmitir
 *	@param	len		cantidad de bytes de datos
 *	@return	1 si OK, 0 si fallo
 *
 *	En caso de falla el status del stack LoRaWan queda en EER34_txStatus.
 */
int EER34_tx(EER34_txMode_t mode, int port, uint8_t *data, int len)
{
	// Copia los datos al buffer
	memcpy(txBuffer, data, len);
	
	// Prepara el request
    lorawanSendReq.buffer = txBuffer;
    lorawanSendReq.bufferLength = len;
	if (mode == EER34_TXMODE_CONF)
		lorawanSendReq.confirmed = LORAWAN_CNF;
	else
		lorawanSendReq.confirmed = LORAWAN_UNCNF;
    lorawanSendReq.port = port;
	
	// Manda el comando de tramsision y recibe la respuesta (que la deja guardada)
    EER34_txStatus = LORAWAN_Send(&lorawanSendReq);
	
	// Si pudo iniciar la transmision larga el timer de timeout
    if (EER34_txStatus == LORAWAN_SUCCESS) {
        SwTimerStart(txTimerId, MS_TO_US(EER34_txTimeout), 
			SW_TIMEOUT_RELATIVE, (void *)txTimerCb, NULL);
		return 1;
	}
	
	return 0;
}

#ifdef CONF_PMM_ENABLE
static void appWakeupCallback(uint32_t sleptDuration)
{
	EES34_exitLowPower();
	HAL_Radio_resources_init();
	sio2host_init();
}
#endif

/** 
 *	Pone el modulo a dormir en modo de bajo consumo
 */
int EER34_sleep(uint32_t time)
{
#ifdef CONF_PMM_ENABLE
	bool deviceResetsForWakeup = false;
	PMM_SleepReq_t sleepReq;
	
	sleepReq.sleepTimeMs = time;
	sleepReq.pmmWakeupCallback = appWakeupCallback;
	sleepReq.sleep_mode = CONF_PMM_SLEEPMODE_WHEN_IDLE;
	if (CONF_PMM_SLEEPMODE_WHEN_IDLE == SLEEP_MODE_STANDBY)
	{
		deviceResetsForWakeup = false;
	}
	if (true == LORAWAN_ReadyToSleep(deviceResetsForWakeup))
	{
		sio2host_deinit();
		HAL_RadioDeInit();
		EES34_enterLowPower();
		if (PMM_SLEEP_REQ_DENIED == PMM_Sleep(&sleepReq))
		{
			EES34_exitLowPower();
			HAL_Radio_resources_init();
			sio2host_init();
			return 0;
		}
		return 1;
	}
	return 0;
#else
	return 0;
#endif
}

/** 
 *	@brief	Lectura de caracter del puerto serie no bloqueante
 *	@return	caracter recibido, o -1 si no hay caracteres
 */
int EER34_getchar(void)
{
	return sio2host_getchar_nowait();
}

/** 
 *	@brief	Inica el lector de lineas del puerto serie
 *	@param	buffer	puntero al buffer de linea
 *	@param	size	tamaño del buffer de linea
 */
void EER34_getLineInit(char *buffer, int size)
{
	lineRdr.buffer = buffer;
	lineRdr.size = size;
	lineRdr.count = 0;
}

/** 
 *	@brief	Lectura de linea del puerto serie no bloqueante
 *	@return	cantidad de caracteres leidos, si no entro una linea todavia
 *
 *	Deja la linea terminada en NULL y sin el caracter de fin de linea.
 */
int EER34_getLine(void)
{
	int c, n;
	
	// Saca todos los caracteres que pueda
	while (1) {
		c = EER34_getchar();
		// Si no hay mas caracteres sale
		if (c <= 0)
			return 0;
		// Si encuentra fin de linea, si habian entrado caracteres
		// cierra la linea y avisa que entro una linea
		if (c == '\r' || c == '\n') {
			if (lineRdr.count) {
				lineRdr.buffer[lineRdr.count++] = 0;
				n = lineRdr.count;
				lineRdr.count = 0;
				return n;
			}
			lineRdr.count = 0;
		}
		// Si no es fin de linea y hay lugar lo mete en el buffer 
		else if (lineRdr.count < lineRdr.size-1)
			lineRdr.buffer[lineRdr.count++] = c;
	}
}

/** 
 *	Callback de respuesta del timer de transmision
 */
static void txTimerCb(void *data)
{
	EER34_statusCallback(EER34_STATUS_TX_TIMEOUT, 0);
}

/** 
 *	Callback de respuesta del timer del tick
 */
static void tickTimerCb(void *data)
{
    SwTimerStart(tickTimerId, MS_TO_US(tickInterval),
	    SW_TIMEOUT_RELATIVE, (void *)tickTimerCb, NULL);
	
	EER34_tickCallback();
}

/** 
 *	Callback de respuesta del Join Request del stack LoRaWan
 */
static void joinResponseCallback(StackRetStatus_t status)
{
	EER34_status_t sts;
	
	if (status == LORAWAN_SUCCESS)	sts = EER34_STATUS_JOIN_SUCCSESS;
	else	sts = EER34_STATUS_JOIN_ERROR;
	
	EER34_statusCallback(sts, status);
}

/** 
 *	Callback de datos del stack LoRaWan
 */
static void appDataCallback(void *appHandle, appCbParams_t *appdata)
{
	EER34_status_t sts = EER34_STATUS_UNKNOWN;
    StackRetStatus_t loraSts = LORAWAN_INVALID_REQUEST;

    SwTimerStop(txTimerId);

    if (appdata->evt == LORAWAN_EVT_RX_DATA_AVAILABLE) {
        loraSts = appdata->param.rxData.status;
		if (loraSts == LORAWAN_SUCCESS) {
		    if((appdata->param.rxData.dataLength > 0) && (appdata->param.rxData.pData != NULL)) {
				EER34_rxDataCallback(appdata->param.rxData.pData[0],
					appdata->param.rxData.pData+1, appdata->param.rxData.dataLength-1);
				return;
			}
			sts = EER34_STATUS_TX_ACK;
		}
	}
	
    else if (appdata->evt == LORAWAN_EVT_TRANSACTION_COMPLETE) {
        loraSts = appdata->param.rxData.status;
		if (loraSts == LORAWAN_SUCCESS || loraSts == LORAWAN_RADIO_SUCCESS)
			sts = EER34_STATUS_TX_SUCCSESS;
	}

	EER34_statusCallback(sts, loraSts);
}

/** 
 *	@brief	Callback de status para la aplicacion
 *	@param	sts			status de EER34
 *	@param	loraSts		status del stack LoRaWan
 */
void __attribute__((weak)) EER34_statusCallback(EER34_status_t sts, StackRetStatus_t LoraSts)
{
}

/** 
 *	@brief	Callback de recepcion de datos la aplicacion
 *	@param	port		numero de puerto
 *	@param	data		puntero a los datos
 *	@param	len			cantidad de bytes de datos
 */
void __attribute__((weak)) EER34_rxDataCallback(int port, uint8_t *data, int len)
{
}

/** 
 *	@brief	Callback del tick de la aplicacion
 */
void __attribute__((weak)) EER34_tickCallback(void)
{
}

/** 
 *	Task de la aplicacion (weak para que compile en el ejemplo original)
 */
SYSTEM_TaskStatus_t  __attribute__((weak)) APP_TaskHandler(void)
{
	EES34_appTask();
	
    SYSTEM_PostTask(APP_TASK_ID);
	
    return SYSTEM_TASK_SUCCESS;
}

/** 
 *	Funcion de inicializacion de la aplicacion
 */
void  __attribute__((weak)) EES34_appInit(void)
{
}

/** 
 *	Task de la aplicacion
 */
void  __attribute__((weak)) EES34_appTask(void)
{
}

/** 
 *	Funcion de procesamiento de la causa de reset
 */
void EES34_reset(void)
{
	enum system_reset_cause rcause = system_get_reset_cause();
	
	EES34_appResetCallback(rcause);
}

/** 
 *	Callback de procesamiento de la causa de reset
 */
void  __attribute__((weak)) EES34_appResetCallback(unsigned int rcause)
{
}

/** 
 *	@brief Callback de entrada en low power
 *
 *  Sirve para que la aplicacion deinicialice y/o apague lo que haga falta
 *  al entrar en el modo de bajo consumo.
 */
void  __attribute__((weak)) EES34_enterLowPower(void)
{
}

/** 
 *	Callback de salida de low power
 *
 *  Sirve para que la aplicacion reponga los recursos al despertar del modo 
 *  de bajo consumo, volviendo a configurar y/o encender los recursos
 *  que deinicializo y/o apago al entrar en el modo de bajo consumo.
 */
void  __attribute__((weak)) EES34_exitLowPower(void)
{
}
