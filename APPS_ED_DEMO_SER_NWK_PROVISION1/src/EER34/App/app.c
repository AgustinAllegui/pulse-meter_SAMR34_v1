/*
 * app.c
 *
 * Created: 22/10/2019 1:44:19 p. m.
 *  Author: gmont_000
 */
#include "..\eer34.h"
#include "inc/logMacros.h"
#include "inc/EER34_nvm.h"
#include "conf_promatix.h"

#include "pmm.h"
#include "radio_driver_hal.h"

#include "EER34_gpio.h"
#include "EER34_adc.h"
#include "EER34_spi.h"
#include "EER34_i2c.h"

#define SAVE_BATTERY_LEVEL 30

// Prototipos de funciones

uint8_t getBatteryLevel(const uint16_t halfVoltage);
void usbParser(const int len);
void payloadParser(uint8_t *rxBuffer, const int len);

// Private variables

uint32_t timer1;	 // Para EER34_tickCallback
uint32_t awakeTimer; // tiempo despierto (1 = 10ms)

#define SEND_BUFFER_LENGTH 6				  // (1 (mnemonico) + 4 (pulseCount)  + 1 (% bateria))
unsigned char sendBuffer[SEND_BUFFER_LENGTH]; // buffer para enviar
volatile uint32_t pulseCount = 0;
uint8_t batteryLevel = 100;
uint32_t period = 30;
//uint32_t period = 3600;

static enum {
	APP_FSM_INIT = 0,
	APP_FSM_LED1,
	APP_FSM_JOIN,
	APP_FSM_JOINING,
	APP_FSM_JOIN_ERROR,
	APP_FSM_JOIN_OK,
	APP_FSM_SAVE_COUNT,
	APP_FSM_PREPARE_PAYLOAD,
	APP_FSM_TX,
	APP_FSM_TX_WAIT,
	APP_FSM_TX_DONE,
	APP_FSM_PRE_SLEEP,
	APP_FSM_SLEEP,
	APP_FSM_POST_SLEEP,
} fsm;

static char line[256];

/** 
 *	@brief	Callback de status para la aplicacion
 *	@param	sts			status de EER34
 *	@param	loraSts		status del stack LoRaWan
 */
void EER34_statusCallback(EER34_status_t sts, StackRetStatus_t LoraSts)
{
	if (sts == EER34_STATUS_JOIN_SUCCSESS)
	{
		if (fsm == APP_FSM_JOINING)
		{
			EER34_Gpio_digitalWrite(PIN_PA14, true);
			timer1 = 50;
			fsm = APP_FSM_JOIN_OK;
		}
	}
	else if (sts == EER34_STATUS_JOIN_ERROR)
	{
		if (fsm == APP_FSM_JOINING)
			fsm = APP_FSM_JOIN_ERROR;
	}
	else if (sts == EER34_STATUS_TX_SUCCSESS)
	{
		if (fsm == APP_FSM_TX_WAIT)
			fsm = APP_FSM_TX_DONE;
	}
	else if (sts == EER34_STATUS_TX_TIMEOUT)
	{
		if (fsm == APP_FSM_TX_WAIT)
			fsm = APP_FSM_TX_DONE;
	}
}

/** 
 *	@brief	Callback de recepcion de datos la aplicacion
 *	@param	port		numero de puerto
 *	@param	data		puntero a los datos
 *	@param	len			cantidad de bytes de datos
 */
void EER34_rxDataCallback(int port, uint8_t *data, int len)
{
	logTrace("Data received");
	payloadParser(data, len);
}

/** 
 *	@brief Callback de entrada en low power
 *
 *  Sirve para que la aplicacion deinicialice y/o apague lo que haga falta
 *  al entrar en el modo de bajo consumo.
 */
void EES34_enterLowPower(void)
{
	// Hay que detener el tick sino no entra en bajo consumo
	EER34_tickStop();
}

/** 
 *	Callback de salida de low power
 *
 *  Sirve para que la aplicacion reponga los recursos al despertar del modo 
 *  de bajo consumo, volviendo a configurar y/o encender los recursos
 *  que deinicializo y/o apago al entrar en el modo de bajo consumo.
 */

uint32_t timeSlept = 0;
void EES34_exitLowPower(const uint32_t slept)
{
	// Vuelve a enceder el tick que lo apago al entrar en bajo consumo
	EER34_tickStart(10);
	awakeTimer = 0;
	logInfo("Time slept: %lu ms", slept);
	timeSlept += slept;
	logDebug("Time slept in total: %lu ms", timeSlept);
}

bool bypassSleep = false;
bool savePulseFlag = false;
// despertar del sleep si se presiona el boton
void buttonCallback(void)
{
	bypassSleep = true;
	savePulseFlag = true;
	PMM_Wakeup();
}

void pulseCallback(void)
{
	pulseCount++;
	PMM_Wakeup();
	logTrace("Pulse");
}

void extintConfigure(void)
{
	logTrace("Inicializando Interrupciones\r\n");
	struct extint_chan_conf pulseConf;
	extint_chan_get_config_defaults(&pulseConf);
	pulseConf.gpio_pin = PIN_PB22;
	pulseConf.gpio_pin_mux = MUX_PB22A_EIC_EXTINT6;
	//pulseConf.gpio_pin_pull = EXTINT_PULL_UP; // quitar pullup al conectar el shield
	pulseConf.gpio_pin_pull = EXTINT_PULL_NONE;
	pulseConf.detection_criteria = EXTINT_DETECT_FALLING;
	pulseConf.filter_input_signal = true;
	pulseConf.enable_async_edge_detection = false;
	extint_chan_set_config(6, &pulseConf);
	extint_register_callback(pulseCallback, 6, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(6, EXTINT_CALLBACK_TYPE_DETECT);

	while (extint_chan_is_detected(6))
	{
		extint_chan_clear_detected(6);
	}

	struct extint_chan_conf bouttonConf;
	extint_chan_get_config_defaults(&bouttonConf);
	bouttonConf.gpio_pin = PIN_PA27;
	bouttonConf.gpio_pin_mux = MUX_PA27A_EIC_EXTINT15;
	bouttonConf.gpio_pin_pull = EXTINT_PULL_NONE;
	bouttonConf.detection_criteria = EXTINT_DETECT_FALLING;
	bouttonConf.filter_input_signal = true;
	bouttonConf.enable_async_edge_detection = false;
	extint_chan_set_config(15, &bouttonConf);
	extint_register_callback(buttonCallback, 15, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(15, EXTINT_CALLBACK_TYPE_DETECT);

	while (extint_chan_is_detected(15))
	{
		extint_chan_clear_detected(15);
	}
}

/**
 * \brief Setect OSC16M as main clock source.
 */
static void set_low_active_power(void)
{
	struct system_gclk_gen_config gclk_conf;
	struct system_clock_source_osc16m_config osc16m_conf;

	/* Switch to new frequency selection and enable OSC16M */
	system_clock_source_osc16m_get_config_defaults(&osc16m_conf);
	osc16m_conf.fsel = CONF_CLOCK_OSC16M_FREQ_SEL;
	osc16m_conf.on_demand = 0;
	osc16m_conf.run_in_standby = CONF_CLOCK_OSC16M_RUN_IN_STANDBY;
	system_clock_source_osc16m_set_config(&osc16m_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_OSC16M);
	while (!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_OSC16M))
		;

	/* Select OSC16M as mainclock */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_OSC16M;
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);
	if (CONF_CLOCK_OSC16M_ON_DEMAND)
	{
		OSCCTRL->OSC16MCTRL.reg |= OSCCTRL_OSC16MCTRL_ONDEMAND;
	}

	system_clock_source_disable(SYSTEM_CLOCK_SOURCE_DFLL);
	system_gclk_chan_disable(OSCCTRL_GCLK_ID_DFLL48);
	system_gclk_gen_disable(GCLK_GENERATOR_1);
	system_switch_performance_level(SYSTEM_PERFORMANCE_LEVEL_0);
}

/** 
 *	Funcion de inicializacion de la aplicacion
 */

void EES34_appInit(void)
{

	// /* Scaling down clock frequency and then Scaling down the performance level */
	set_low_active_power();

	static volatile int res;

	uint8_t devEuix[] = PROMATIX_DEV_EUI;
	uint8_t appEuix[] = PROMATIX_APP_EUI;
	uint8_t appKeyx[] = PROMATIX_APP_KEY;
	uint8_t frequencySubBand = PROMATIX_SUB_BAND;

	logWarning("-----------------------------------");
	logWarning("EESAMR34");
	logWarning("Initializing\r\n");

	logInfo("Dev EUI: %02X %02X %02X %02X %02X %02X %02X %02X",
			devEuix[0], devEuix[1], devEuix[2], devEuix[3],
			devEuix[4], devEuix[5], devEuix[6], devEuix[7]);

	logInfo("App EUI: %02X %02X %02X %02X %02X %02X %02X %02X ",
			appEuix[0], appEuix[1], appEuix[2], appEuix[3],
			appEuix[4], appEuix[5], appEuix[6], appEuix[7]);

	logInfo("App KEY: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
			appKeyx[0], appKeyx[1], appKeyx[2], appKeyx[3],
			appKeyx[4], appKeyx[5], appKeyx[6], appKeyx[7],
			appKeyx[8], appKeyx[9], appKeyx[10], appKeyx[11],
			appKeyx[12], appKeyx[13], appKeyx[14], appKeyx[15]);

	logInfo("Frequency Sub Band: %u", frequencySubBand);

	// Este seteo debe ir primero porque inicia el stack LoRaWan

	res = EER34_setBand(ISM_AU915, frequencySubBand);

	// Estos seteos pueden ir en cualquier orden pero siempre
	// despues de setear la banda (sino dan error)
	res = EER34_setDevEui(devEuix);
	res = EER34_setAppEui(appEuix);
	res = EER34_setAppKey(appKeyx);
	res = EER34_setDeviceClass(CLASS_A);
	res = EER34_setAdr(EER34_ADR_ON);

	// Arranca tick de 10ms
	EER34_tickStart(10); // arranca tick de 10ms

	logInfo("Initialization Done\r\n\r\n");

	//============================================================
	// Initialize pins

	EER34_Gpio_pinMode(PIN_PA14, OUTPUT); // LED 2
	//EER34_Gpio_digitalWrite(PIN_PA14, false);
	//EER34_Gpio_pinMode(PIN_PA07, OUTPUT); // LED 5
	//#if LOG_LEVEL >= DEBUG_LEVEL
	
	//#endif

	// EER34_Gpio_pinMode ( PIN_PA27 , INPUT  );			// SW2 with external pull-up
	extintConfigure();
	// recover count
	pulseCount = 0;

	//============================================================

	EER34_getLineInit(line, sizeof(line));

	//============================================================
	// confiure EEPROM
	logTrace("Initializing EEPROM");
	configure_eeprom();

	period = getSavedPeriod();
	pulseCount = getSavedPulseCount();

	logInfo("Period: %lu", period);
	logInfo("Pulses counted: %lu", pulseCount);
}

/** 
 *	@brief	Callback del tick de la aplicacion
 */
void EER34_tickCallback(void)
{
	awakeTimer++;
	if (timer1)
		timer1--;
}

/** 
 *	Task de la aplicacion
 */
void EES34_appTask(void)
{

	static bool isJoined = false;
	static uint8_t joinAttempts;
	static uint8_t txAttemptCount = 0;

	// Procesar datos recibidos por USB
	{
		int usbLen = EER34_getLine();
		if (usbLen)
			usbParser(usbLen);
	}

	//--------------------------------------------------------
	// Manejo de maquina de estados.

	switch (fsm)
	{
	case APP_FSM_INIT:
	{
		logTrace("INIT");
		joinAttempts = 32;
		EER34_Gpio_digitalWrite(PIN_PA14, true);
		timer1 = 50;
		fsm = APP_FSM_LED1;
		break;
	}
	case APP_FSM_LED1:
	{
		if(timer1 == 0){
			logTrace("Apagar led");
			EER34_Gpio_digitalWrite(PIN_PA14, false);
			fsm = APP_FSM_JOIN;
		}
		break;
	}
	case APP_FSM_JOIN_ERROR:
	{
		logError("Join failed");
	}
	case APP_FSM_JOIN:
	{
		logTrace("Check join");
		if (isJoined || joinAttempts <= 0)
		{
			fsm = APP_FSM_SAVE_COUNT;
			break;
		}
		joinAttempts--;

		if (EER34_joinOTAA())
		{
			logInfo("Joining...");
			fsm = APP_FSM_JOINING;
		}
		else
		{
			logError("Failed to send join request");
			fsm = APP_FSM_JOIN_ERROR;
		}
		break;
	}
	case APP_FSM_JOINING:
	{
		break;
	}
	case APP_FSM_JOIN_OK:
	{
		if(!timer1){
			logInfo("Join ok");
			isJoined = true;
			EER34_Gpio_digitalWrite(PIN_PA14, false);
			fsm = APP_FSM_SAVE_COUNT;	
		}
		break;
	}
	case APP_FSM_SAVE_COUNT:
	{
		logTrace("Save count");
		//! leer nivel de bateria.
		//uint16_t batteryRead = EER34_Adc_digitalRead();
		logDebug("Reading ADC 8 times");

		// Initialize adc
		logInfo("Initializing ADC");
		EER34_Adc_startAdc();

		uint16_t batteryRead = 0;
		for (int i = 0; i < 8; i++)
		{
			batteryRead += EER34_Adc_digitalRead();
			if (i < 2)
			{
				batteryRead = 0;
			}
		}
		logDebug("ADC read x6 %u", batteryRead);
		logInfo("Deinitializing ADC");
		EER34_Adc_disableAdc();
		batteryRead = batteryRead / 6;

		logDebug("ADC read %u", batteryRead);

		batteryLevel = getBatteryLevel(batteryRead);
		//batteryLevel = pulseCount%101;
		logInfo("Battery level %u%", batteryLevel);
		//! si nivel de bateria baja, guardar en flash.

		if (batteryLevel < SAVE_BATTERY_LEVEL || savePulseFlag)
		{
			savePulseFlag = false;
			savePulseCount(pulseCount);
		}

		fsm = APP_FSM_PREPARE_PAYLOAD;
		break;
	}
	case APP_FSM_PREPARE_PAYLOAD:
	{
		logTrace("Prepare payload");
		if (isJoined)
		{
			// preparar frame
			sendBuffer[0] = 'S';
			uint32_t pulseCountToSend = pulseCount;
			logInfo("Pulses counted %lu", pulseCountToSend);
			sendBuffer[4] = (char)((pulseCountToSend & 0x000000FF));
			pulseCountToSend = pulseCountToSend >> 8;
			sendBuffer[3] = (char)((pulseCountToSend & 0x000000FF));
			pulseCountToSend = pulseCountToSend >> 8;
			sendBuffer[2] = (char)((pulseCountToSend & 0x000000FF));
			pulseCountToSend = pulseCountToSend >> 8;
			sendBuffer[1] = (char)((pulseCountToSend & 0x000000FF));

			sendBuffer[5] = (char)(batteryLevel);

#if LOG_LEVEL >= DEBUG_LEVEL
			printf("[DEBUG] Payload to be send: [ ");
			for (int i = 0; i < SEND_BUFFER_LENGTH; i++)
			{
				printf("%02X ", sendBuffer[i]);
			}
			printf("]\r\n");
#endif

			fsm = APP_FSM_TX;
		}
		else
		{
			timer1 = 50;
			fsm = APP_FSM_PRE_SLEEP;
		}
		break;
	}
	case APP_FSM_TX:
	{
		if (txAttemptCount < 5)
		{
			if (!timer1)
			{
				txAttemptCount++;
				logTrace("TX");
				logInfo("Attempt %u to send", txAttemptCount);
				if (EER34_tx(EER34_TXMODE_UNCONF, 1, sendBuffer, SEND_BUFFER_LENGTH))
				{
					logInfo("Transmitting");
					fsm = APP_FSM_TX_WAIT;
				}
				else
				{
					logInfo("TX failed");
					timer1 = 50;
				}
			}
		}
		else
		{
			fsm = APP_FSM_TX_DONE;
		}
		break;
	}
	case APP_FSM_TX_WAIT:
	{
		break;
	}
	case APP_FSM_TX_DONE:
	{
		logTrace("TX DONE");
		txAttemptCount = 0;
		timer1 = 50;
		fsm = APP_FSM_PRE_SLEEP;
		break;
	}
	case APP_FSM_PRE_SLEEP:
	{
		// deinit radio
		logTrace("Deinit Radio");
		HAL_RadioDeInit();
		HAL_TCXOPowerOff();
		fsm = APP_FSM_SLEEP;
		break;
	}
	case APP_FSM_SLEEP:
	{
		if (!timer1)
		{

			logTrace("SLEEP");
			// calcular tiempo a dormir

			logDebug("Tiempo despierto %lu", awakeTimer / 100);
			//logDebug("Tiempo desde sleep %lu", US_TO_MS(SLEEP_TICKS_TO_US(SleepTimerGetElapsedTime())));

			uint32_t timeToSleep = (period * 1000) - timeSlept - awakeTimer * 10;
			logInfo("About to sleep %lu ms", timeToSleep);
			if (EER34_sleep(timeToSleep))
			{
				logInfo("Slept OK, woken-up!\r\n");
				if (timeSlept > ((period - 4)) * 1000 || bypassSleep)
				{
					bypassSleep = false;
					timeSlept = 0;
					fsm = APP_FSM_POST_SLEEP;
				}
			}
			else
			{
				logError("Sleep failed\r\n");
				timer1 = 50;
			}
			// esperar 30 segundos en vez de sleep
			//int delayInSeconds = 30;
			//timer1 = delayInSeconds*100;
			//logDebug("Simulando sleep");
			//logInfo("Retardo de %u seg\r\n", period);

			joinAttempts = 8;
		}
		break;
	}

	case APP_FSM_POST_SLEEP:
	{
		// Init radio
		logTrace("Init radio");
		HAL_Radio_resources_init();
		HAL_TCXOPowerOn();
		fsm = APP_FSM_JOIN;
		break;
	}

	default:
		break;
	}
}

/** 
 *	Callback de procesamiento de la causa de reset
 */
void EES34_appResetCallback(unsigned int rcause)
{
	logWarning("Last reset cause:");
	if (rcause & (1 << 6))
	{
		logWarning("System Reset Request\r\n");
	}
	if (rcause & (1 << 5))
	{
		logWarning("Watchdog Reset\r\n");
	}
	if (rcause & (1 << 4))
	{
		logWarning("External Reset\r\n");
	}
	if (rcause & (1 << 2))
	{
		logWarning("Brown Out 33 Detector Reset\r\n");
	}
	if (rcause & (1 << 1))
	{
		logWarning("Brown Out 12 Detector Reset\r\n");
	}
	if (rcause & (1 << 0))
	{
		logWarning("Power-On Reset\r\n");
	}
}

/**
 * @brief Obtener el nivel de bateria a partir de la mitad del voltaje actual.
 * 
 * @param halfVoltage Mitad del voltaje de la bateria.
 * @return Nivel de bateria (0 - 100)
 */
uint8_t getBatteryLevel(const uint16_t halfVoltage)
{
	//const uint16_t minLevel = 2000; // medicion para 3.3v
	//const uint16_t maxLevel = 2500; // medicion para 4.2v

	// Sensado modificado para usar referencia interna 1v
	const uint16_t minLevel = 3143; // medicion para 3.3v
	const uint16_t maxLevel = 3765; // medicion para 4.2v

	float percentage = (halfVoltage - minLevel);
	percentage = percentage / (maxLevel - minLevel);
	percentage *= 100;

	if (percentage > 100)
		percentage = 100;
	else if (percentage < 0)
		percentage = 0;

	return (uint8_t)percentage;
}

char char2hex(const char caracter)
{
	if (caracter >= '0' && caracter <= '9')
		return caracter - '0';
	else if (caracter >= 'A' && caracter <= 'F')
		return caracter + 10 - 'A';
	else if (caracter >= 'a' && caracter <= 'f')
		return caracter + 10 - 'a';
	else
		return 0;
}

void usbParser(const int len)
{
	logInfo("Send through USB: %s", line);
	switch (line[0])
	{
	case 'L':
	{
		char rxBuffer[(len - 1) / 2];
		for (int i = 1; i < len - 1; i += 2)
		{
			char c = char2hex(line[i]) << 4;
			c |= char2hex(line[i + 1]);
			logDebug("i: %2i, c: %02X", i, c);
			rxBuffer[(i - 1) / 2] = c;
		}
		payloadParser(rxBuffer, (len - 1) / 2);
		break;
	}
	default:
		break;
	}
}

void payloadParser(uint8_t *rxBuffer, const int len)
{
	/**	//! Parsear datos recibidos
	 * [x] Tiempo entre transmisiones
	 * [ ] ?Hora actual
	 * [x] Contador de pulsos manual
	 * [x] Pulsos por unidad
	 */

	logTrace("payloadParser");
#if LOG_LEVEL >= INFO_LEVEL
	printf("[INFO] Recived data: ");
	for (int i = 0; i < len; i++)
	{
		printf("%02X ", rxBuffer[i]);
	}
	printf("\r\n");
#endif

	switch (rxBuffer[0])
	{
	case 'T':
	{
		if (len < 3)
			break;
		logInfo("Changing period");
		uint32_t receivedPeriod = rxBuffer[1];
		receivedPeriod = receivedPeriod << 8;
		receivedPeriod |= rxBuffer[2];
		receivedPeriod = receivedPeriod << 8;
		receivedPeriod |= rxBuffer[3];
		receivedPeriod = receivedPeriod << 8;
		receivedPeriod |= rxBuffer[4];

		period = receivedPeriod;
		if (period < 30)
			period = 30;
		else if (period > 4294965) // periodo mayor a (2^32)/1000
			period = 42949671;
		logDebug("Period: %lu seconds", period);

		savePeriod(period);
		break;
	}
	case 'P':
	{
		if (len < 5)
			break;
		logInfo("Manualy setting pulse count");
		uint32_t receivedPulses = rxBuffer[1];
		receivedPulses = receivedPulses << 8;
		receivedPulses |= rxBuffer[2];
		receivedPulses = receivedPulses << 8;
		receivedPulses |= rxBuffer[3];
		receivedPulses = receivedPulses << 8;
		receivedPulses |= rxBuffer[4];
		logDebug("Pulse count: %lu", receivedPulses);

		clearPulseCount();
		savePulseCount(receivedPulses);
		pulseCount = receivedPulses;
		break;
	}
	case 'A':
	{
		if (len < 5)
			break;
		logInfo("Manualy add pulses to count");
		uint32_t receivedPulses = rxBuffer[1];
		receivedPulses = receivedPulses << 8;
		receivedPulses |= rxBuffer[2];
		receivedPulses = receivedPulses << 8;
		receivedPulses |= rxBuffer[3];
		receivedPulses = receivedPulses << 8;
		receivedPulses |= rxBuffer[4];
		logDebug("Received pules: %lu", receivedPulses);
		pulseCount += receivedPulses;
		clearPulseCount();
		savePulseCount(pulseCount);
		break;
	}
	default:
		break;
	}
}
