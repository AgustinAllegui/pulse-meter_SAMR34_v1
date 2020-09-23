/*
 * app.c
 *
 * Created: 22/10/2019 1:44:19 p. m.
 *  Author: gmont_000
 */
#include "..\eer34.h"
#include "inc/logMacros.h"

#include "EER34_gpio.h"
#include "EER34_adc.h"
#include "EER34_spi.h"
#include "EER34_i2c.h"

// Prototipos de funciones

uint8_t getBatteryLevel(const uint16_t halfVoltage);

// Private variables

int timer1; // Para EER34_tickCallback

#define SEND_BUFFER_LENGTH 6				  // (1 (mnemonico) + 4 (pulseCount) + 1 (% bateria))
unsigned char sendBuffer[SEND_BUFFER_LENGTH]; // buffer para enviar
volatile uint32_t pulseCount = 0;
uint8_t batteryLevel = 100;

static enum {
	APP_FSM_INIT = 0,
	APP_FSM_JOIN,
	APP_FSM_JOINING,
	APP_FSM_JOIN_ERROR,
	APP_FSM_JOIN_OK,
	APP_FSM_SAVE_COUNT,
	APP_FSM_PREPARE_PAYLOAD,
	APP_FSM_TX,
	APP_FSM_TX_WAIT,
	APP_FSM_TX_DONE,
	APP_FSM_SLEEP,
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
			fsm = APP_FSM_JOIN_OK;
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
	/**	//! Parsear datos recibidos
	 * Tiempo entre transmisiones
	 * Hora actual
	 * 
	 * 
	 */
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
void EES34_exitLowPower(void)
{
	// Vuelve a enceder el tick que lo apago al entrar en bajo consumo
	EER34_tickStart(10);
}

void extintCallback(void)
{
	pulseCount++;
}

#define IRQ_PIN PIN_PA27
#define IRQ_MUX MUX_PA27A_EIC_EXTINT15
#define IRQ_CHAN 15

void extintConfigure(void)
{
	logTrace("Inicializando Interrupciones\r\n");
	struct extint_chan_conf chanConf;
	extint_chan_get_config_defaults(&chanConf);
	chanConf.gpio_pin = IRQ_PIN;
	chanConf.gpio_pin_mux = IRQ_MUX;
	chanConf.gpio_pin_pull = EXTINT_PULL_UP;
	chanConf.detection_criteria = EXTINT_DETECT_FALLING;
	chanConf.filter_input_signal = true;
	chanConf.enable_async_edge_detection = false;
	extint_chan_set_config(IRQ_CHAN, &chanConf);
	extint_register_callback(extintCallback, IRQ_CHAN, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(IRQ_CHAN, EXTINT_CALLBACK_TYPE_DETECT);

	while (extint_chan_is_detected(IRQ_CHAN))
	{
		extint_chan_clear_detected(IRQ_CHAN);
	}
}

/** 
 *	Funcion de inicializacion de la aplicacion
 */

void EES34_appInit(void)
{
	static volatile int res;

	uint8_t devEuix[] = {0xde, 0xaf, 0xfa, 0xce, 0xde, 0xaf, 0x55, 0x20};
	uint8_t appEuix[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
	uint8_t appKeyx[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};

	logWarning("-----------------------------------");
	logWarning("EESAMR34");
	logWarning("Initializing\r\n");

	// Este seteo debe ir primero porque inicia el stack LoRaWan
	res = EER34_setBand(ISM_AU915, 2);

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
	EER34_Gpio_pinMode(PIN_PA07, OUTPUT); // LED 5

	// EER34_Gpio_pinMode ( PIN_PA27 , INPUT  );			// SW2 with external pull-up
	extintConfigure();

	// Initialize adc
	{
		struct adc_config config_adc;

		adc_get_config_defaults(&config_adc);
#if LOG_LEVEL >= DEBUG_LEVEL
		if (config_adc.positive_input == ADC_POSITIVE_INPUT_PIN6)
			logDebug("ADC to battery");
		else if (config_adc.positive_input == ADC_POSITIVE_INPUT_PIN6)
			logDebug("ADC to pin1");
#endif
		config_adc.positive_input = ADC_POSITIVE_INPUT_PIN6;
		//config_adc.positive_input = ADC_POSITIVE_INPUT_PIN8;

		adc_init(&adc_instance, ADC, &config_adc);
		adc_enable(&adc_instance);
	}

	//	// Inilialize SPI
	EER34_configureSpiMaster(100000, SPI_DATA_ORDER_MSB, SPI_TRANSFER_MODE_0);

	// Initialize I2C
	EER34_I2C_begin();

	//============================================================

	EER34_getLineInit(line, sizeof(line));
}

/** 
 *	@brief	Callback del tick de la aplicacion
 */
void EER34_tickCallback(void)
{
	if (timer1)
		timer1--;
}

static void ToggleLed2(void)
{
	static bool led2 = 0;

	led2 = led2 ^ 1;

	if (led2)
		EER34_Gpio_digitalWrite(PIN_PA14, true);
	else
		EER34_Gpio_digitalWrite(PIN_PA14, false);
}

static void ToggleLed5(void)
{
	static bool led5 = 0;

	led5 = led5 ^ 1;

	if (led5)
		EER34_Gpio_digitalWrite(PIN_PA07, true);
	else
		EER34_Gpio_digitalWrite(PIN_PA07, false);
}

static void TestSPI(void)
{
	uint8_t buffer_spi_tx[20] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13};
	uint8_t buffer_spi_rx[20] = {0x00};

	memcpy_ram2ram(buffer_spi_rx, buffer_spi_tx, 20);

	EER34_spiStartTransaction();
	EER34_spiTransferBuffer(buffer_spi_tx, sizeof(buffer_spi_tx));
	EER34_spiEndTransaction();

	if (memcmp_ram2ram(buffer_spi_tx, buffer_spi_rx, 20) == 0)
	{
		ToggleLed5();
	}
}

void TestI2C(void)
{
#define SLAVE_ADDRESS 0x28

	uint8_t buffer_i2c_tx[20] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13};
	uint8_t buffer_i2c_rx[20] = {0};

	EER34_I2C_Write(SLAVE_ADDRESS, buffer_i2c_tx, 20);

	EER34_I2C_Read(SLAVE_ADDRESS, buffer_i2c_rx, 20);
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
	if (EER34_getLine())
		logInfo("You typed: %s\r\n", line);

	//--------------------------------------------------------
	// Manejo de maquina de estados.

	switch (fsm)
	{
	case APP_FSM_INIT:
	{
		logTrace("INIT");
		joinAttempts = 32;
		fsm = APP_FSM_JOIN;
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
		logTrace("Join ok");
		isJoined = true;
		fsm = APP_FSM_SAVE_COUNT;
		break;
	}
	case APP_FSM_SAVE_COUNT:
	{
		logTrace("Save count");
		//! leer nivel de bateria.
		uint16_t batteryRead = EER34_Adc_digitalRead();
		logDebug("ADC read %d", batteryRead);
		batteryLevel = getBatteryLevel(batteryRead);
		logInfo("Battery level %u%", batteryLevel);
		//! si nivel de bateria baja, guardar en flash.
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
			timer1 = 3000;
			fsm = APP_FSM_SLEEP;
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
		timer1 = 3000;
		fsm = APP_FSM_SLEEP;
		break;
	}
	case APP_FSM_SLEEP:
	{
		if (!timer1)
		{

			logTrace("SLEEP");
			//! implementar sleep

			// esperar 30 segundos en vez de sleep
			//int delayInSeconds = 30;
			//timer1 = delayInSeconds*100;
			logInfo("Retardo de 30 seg\r\n");

			joinAttempts = 8;

			fsm = APP_FSM_JOIN;
		}
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
	const uint16_t minLevel = 3217; // medicion estimada para 3.3v
	const uint16_t maxLevel = 4095; // medicion para 4.2v

	float percentage = (halfVoltage - minLevel);
	percentage = percentage / (maxLevel - minLevel);
	percentage *= 100;
	
	if (percentage > 100)
		percentage = 100;

	return (uint8_t)percentage;
}
