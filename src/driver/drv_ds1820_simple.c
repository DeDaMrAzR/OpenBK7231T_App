#include "drv_ds1820_simple.h"
#include "../hal/hal_generic.h"
#if PLATFORM_ESPIDF
#include "freertos/task.h"
#define noInterrupts() portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;taskENTER_CRITICAL(&mux)
#define interrupts() taskEXIT_CRITICAL(&mux)
#elif LINUX
#define noInterrupts() 
#define interrupts() 
#else
#include <task.h>
#define noInterrupts() taskENTER_CRITICAL()
#define interrupts() taskEXIT_CRITICAL()
#endif

static uint8_t dsread = 0;
static int Pin = -1;
static int t = -127;
static int errcount = 0;
static int lastconv; // secondsElapsed on last successfull reading
static uint8_t ds18_family = 0;
static int ds18_conversionPeriod = 0;

static int DS1820_DiscoverFamily();

#define DS1820_LOG(x, fmt, ...) addLogAdv(LOG_##x, LOG_FEATURE_SENSOR, "DS1820[%i] - " fmt, Pin, ##__VA_ARGS__)

// usleep adopted from DHT driver
static void usleepds(int r)
{
	HAL_Delay_us(r);
}

// add some "special timing" for Beken - works w/o and with powerSave 1 for me
static void usleepshort(int r) //delay function do 10*r nops, because rtos_delay_milliseconds is too much
{
#if PLATFORM_BEKEN
	int newr = r / (3 * g_powersave + 1);		// devide by 4 if powerSave set to 1
	for(volatile int i = 0; i < newr; i++)
	{
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
		//__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
		__asm__("nop\nnop\nnop\nnop");
	}

#else
	usleepds(r);
#endif
}

static void usleepmed(int r) //delay function do 10*r nops, because rtos_delay_milliseconds is too much
{
#if PLATFORM_BEKEN
	int newr = 10 * r / (10 + 5 * g_powersave);		// devide by 1.5 powerSave set to 1
	for(volatile int i = 0; i < newr; i++)
	{
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");	// 5
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
	}

#else
	usleepds(r);
#endif
}

static void usleeplong(int r) //delay function do 10*r nops, because rtos_delay_milliseconds is too much
{
#if PLATFORM_BEKEN
	int newr = 10 * r / (10 + 5 * g_powersave);		// devide by 1.5 powerSave set to 1
	for(volatile int i = 0; i < newr; i++)
	{
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
		//		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");	// 5
		__asm__("nop\nnop\nnop\nnop\nnop");	// 5
	}

#else
	usleepds(r);
#endif
}

/*

timing numbers and general code idea from

https://www.analog.com/en/resources/technical-articles/1wire-communication-through-software.html

Parameter 	Speed 		Recommended (µs)
A 		Standard 	6
B 		Standard 	64
C 		Standard 	60
D 		Standard 	10
E 		Standard 	9
F 		Standard 	55
G 		Standard 	0
H 		Standard 	480
I 		Standard 	70
J 		Standard 	410

*/

#define OWtimeA	6
#define OWtimeB	64
#define OWtimeC	60
#define OWtimeD	10
#define OWtimeE	9
#define OWtimeF	55
#define OWtimeG	0
#define OWtimeH	480
#define OWtimeI	70
#define OWtimeJ	410

#define READ_ROM 0x33
#define SKIP_ROM 0xCC
#define CONVERT_T 0x44
#define READ_SCRATCHPAD 0xBE
#define WRITE_SCRATCHPAD 0x4E

static int OWReset(int Pin)
{
	int result;

	//usleep(OWtimeG);
	HAL_PIN_Setup_Output(Pin);
	HAL_PIN_SetOutputValue(Pin, 0); // Drives DQ low
	usleeplong(OWtimeH);
	HAL_PIN_SetOutputValue(Pin, 1); // Releases the bus
	usleepmed(OWtimeI);
	HAL_PIN_Setup_Input(Pin);
	result = HAL_PIN_ReadDigitalInput(Pin) ^ 0x01; // Sample for presence pulse from slave
	usleeplong(OWtimeJ); // Complete the reset sequence recovery
	return result; // Return sample presence pulse result
}

//-----------------------------------------------------------------------------
// Send a 1-Wire write bit. Provide 10us recovery time.
//-----------------------------------------------------------------------------
static void OWWriteBit(int Pin, int bit)
{
	if(bit)
	{
		// Write '1' bit
		HAL_PIN_Setup_Output(Pin);
		noInterrupts();
		HAL_PIN_SetOutputValue(Pin, 0); // Drives DQ low
		usleepshort(OWtimeA);
		HAL_PIN_SetOutputValue(Pin, 1); // Releases the bus
		interrupts();	// hope for the best for the following timer and keep CRITICAL as short as possible
		usleepmed(OWtimeB); // Complete the time slot and 10us recovery
	}
	else
	{
		// Write '0' bit
		HAL_PIN_Setup_Output(Pin);
		noInterrupts();
		HAL_PIN_SetOutputValue(Pin, 0); // Drives DQ low
		usleepmed(OWtimeC);
		HAL_PIN_SetOutputValue(Pin, 1); // Releases the bus
		interrupts();	// hope for the best for the following timer and keep CRITICAL as short as possible
		usleepshort(OWtimeD);
	}
}

//-----------------------------------------------------------------------------
// Read a bit from the 1-Wire bus and return it. Provide 10us recovery time.
//-----------------------------------------------------------------------------
static int OWReadBit(int Pin)
{
	int result;

	noInterrupts();
	HAL_PIN_Setup_Output(Pin);
	HAL_PIN_SetOutputValue(Pin, 0); // Drives DQ low
	usleepshort(OWtimeA);
	HAL_PIN_SetOutputValue(Pin, 1); // Releases the bus
	usleepshort(OWtimeE);
	HAL_PIN_Setup_Input(Pin);
	result = HAL_PIN_ReadDigitalInput(Pin); // Sample for presence pulse from slave
	interrupts();	// hope for the best for the following timer and keep CRITICAL as short as possible
	usleepmed(OWtimeF); // Complete the time slot and 10us recovery
	return result;
}

//-----------------------------------------------------------------------------
// Poll if DS1820 temperature conversion is complete
//-----------------------------------------------------------------------------
static int DS1820TConversionDone(int Pin)
{
	// Write '1' bit
	OWWriteBit(Pin, 1);
	// check for '1' - conversion complete (will be '0' else)
	return OWReadBit(Pin);
}


//-----------------------------------------------------------------------------
// Write 1-Wire data byte
//-----------------------------------------------------------------------------
static void OWWriteByte(int Pin, int data)
{
	int loop;

	// Loop to write each bit in the byte, LS-bit first
	for(loop = 0; loop < 8; loop++)
	{
		OWWriteBit(Pin, data & 0x01);

		// shift the data byte for the next bit
		data >>= 1;
	}
}

//-----------------------------------------------------------------------------
// Read 1-Wire data byte and return it
//-----------------------------------------------------------------------------
static int OWReadByte(int Pin)
{
	int loop, result = 0;

	for(loop = 0; loop < 8; loop++)
	{
		// shift the result to get it ready for the next bit
		result >>= 1;

		// if result is one, then set MS bit
		if(OWReadBit(Pin))
			result |= 0x80;
	}
	return result;
}

//-----------------------------------------------------------------------------
// Write a 1-Wire data byte and return the sampled result.
//-----------------------------------------------------------------------------
static int OWTouchByte(int Pin, int data)
{
	int loop, result = 0;

	for(loop = 0; loop < 8; loop++)
	{
		// shift the result to get it ready for the next bit
		result >>= 1;

		// If sending a '1' then read a bit else write a '0'
		if(data & 0x01)
		{
			if(OWReadBit(Pin))
				result |= 0x80;
		}
		else
			OWWriteBit(Pin, 0);

		// shift the data byte for the next bit
		data >>= 1;
	}
	return result;
}

// quicker CRC with lookup table
// based on code found here: https://community.st.com/t5/stm32-mcus-security/use-stm32-crc-to-compare-ds18b20-crc/m-p/333749/highlight/true#M4690
// Dallas 1-Wire CRC Test App -
//  x^8 + x^5 + x^4 + 1 0x8C (0x131)

static uint8_t Crc8CQuick(uint8_t* Buffer, uint8_t Size)
{
	// Nibble table for polynomial 0x8C
	static const uint8_t CrcTable[] =
	{
		0x00,0x9D,0x23,0xBE,0x46,0xDB,0x65,0xF8,
		0x8C,0x11,0xAF,0x32,0xCA,0x57,0xE9,0x74
	};
	uint8_t Crc = 0x00;
	while(Size--)
	{
		Crc ^= *Buffer++; // Apply Data    
		Crc = (Crc >> 4) ^ CrcTable[Crc & 0x0F]; // Two rounds of 4-bits
		Crc = (Crc >> 4) ^ CrcTable[Crc & 0x0F];
	}
	return(Crc);
}

int DS1820_getTemp()
{
	return t;
}

static commandResult_t Cmd_SetResolution(const void* context, const char* cmd, const char* args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	int arg = Tokenizer_GetArgInteger(0);
	if (arg > 12 || arg < 9)
		return CMD_RES_BAD_ARGUMENT;

	if (ds18_family != 0x28) {
		DS1820_LOG(ERROR, "DS1820_SetResolution not supported by sensor");
		return CMD_RES_UNKNOWN_COMMAND;
	}

	uint8_t cfg = arg;
	cfg = cfg - 9;
	cfg = cfg * 32;
	cfg |= 0x1F;

	if(OWReset(Pin) == 0)
	{
		DS1820_LOG(ERROR, "WriteScratchpad Reset failed");
		return CMD_RES_ERROR;
	}

	OWWriteByte(Pin, SKIP_ROM);
	OWWriteByte(Pin, WRITE_SCRATCHPAD); //Write Scratchpad command
	OWWriteByte(Pin, 0x7F); //TH
	OWWriteByte(Pin, 0x80); //TL
	OWWriteByte(Pin, cfg);  //CFG

	//temperature conversion was interrupted
	dsread = 0;

	return CMD_RES_OK;
}

// startDriver DS1820 [conversionPeriod (seconds) - default 15]
void DS1820_driver_Init()
{
	ds18_conversionPeriod = Tokenizer_GetArgIntegerDefault(1, 15);
	lastconv = 0;
	dsread = 0;
	ds18_family = 0;

	//cmddetail:{"name":"DS1820_SetResolution","args":"[int]",
	//cmddetail:"descr":"Sets resolution for connected DS1820 sensor (9/10/11/12 bits)",
	//cmddetail:"fn":"Cmd_SetResolution","file":"drv/drv_ds1820_simple.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("DS1820_SetResolution", Cmd_SetResolution, NULL);

	//Find PIN and check device so DS1820_SetResolution could be used in autoexec.bat
	Pin = PIN_FindPinIndexForRole(IOR_DS1820_IO, -1);
	if (Pin >= 0)
		DS1820_DiscoverFamily();
};

void DS1820_AppendInformationToHTTPIndexPage(http_request_t* request)
{
	hprintf255(request, "<h5>DS1820 Temperature: %.2f C (read %i secs ago)</h5>", (float)t / 100, g_secondsElapsed - lastconv);
}

static int DS1820_DiscoverFamily()
{
	if(!OWReset(Pin))
	{
		DS1820_LOG(DEBUG, "Discover Reset failed");
		return 0;
	}

	// Read ROM
	uint8_t ROM[8];
	OWWriteByte(Pin, READ_ROM);
	for(int i = 0; i < 8; i++)
	{
		ROM[i] = OWReadByte(Pin);
	}

	// Check CRC
	uint8_t crc = Crc8CQuick(ROM, 7);
	if(crc != ROM[7])
	{
		// This might mean bad signal integrity or multiple 1-wire devices on the bus
		DS1820_LOG(DEBUG, "Discover CRC failed (CRC=%x != calculated:%x)", ROM[7], crc);
		return 0;
	}

	// Check family
	uint8_t family = ROM[0];
	if(family == 0x10 || family == 0x28)
	{
		ds18_family = family;
		DS1820_LOG(INFO, "Discover Family - discovered %x", family);
		return 1;
	}
	else
	{
		DS1820_LOG(DEBUG, "Discover Family %x not supported", family);
		return 0;
	}
}

void DS1820_OnEverySecond()
{
	uint8_t scratchpad[9], crc;
	int16_t raw;

	// for now just find the pin used
	Pin = PIN_FindPinIndexForRole(IOR_DS1820_IO, -1);

	// only if pin is set
	if(Pin < 0)
		return;
	
	//Temperature measurement is done in two repeatable steps. 
	// Step 1 - dsread = 0. Sensor requested to do temperature conversion.
	//          That requires some time - 15-100-750ms, depending on sensor family/vendor.
	//          However, time between steps is always one second.
	// Step 2 - dsread = 1. Sensor finished conversion, requesting conversion result.

	// request temp if conversion was requested two seconds after request
	// if (dsread == 1 && g_secondsElapsed % 5 == 2) {
	// better if we don't use parasitic power, we can check if conversion is ready
	if(dsread == 1)
	{
		if (!DS1820TConversionDone(Pin))
			return;

		if(OWReset(Pin) == 0)
		{
			DS1820_LOG(ERROR, "Read Reset failed");
			return;
		}

		OWWriteByte(Pin, SKIP_ROM);
		OWWriteByte(Pin, READ_SCRATCHPAD);

		for(int i = 0; i < 9; i++)
		{
			scratchpad[i] = OWReadByte(Pin);
		}

		crc = Crc8CQuick(scratchpad, 8);
		if(crc != scratchpad[8])
		{
			DS1820_LOG(ERROR, "Read CRC=%x != calculated:%x (errcount=%i)", scratchpad[8], crc, errcount);
			DS1820_LOG(ERROR, "Scratchpad Data Read: %x %x %x %x %x %x %x %x %x",
				scratchpad[0], scratchpad[1], scratchpad[2], scratchpad[3], scratchpad[4],
				scratchpad[5], scratchpad[6], scratchpad[7], scratchpad[8]);

			errcount++;
			if(errcount > 5) dsread = 0; // retry afer 5 failures
			
			return;
		}

		raw = (scratchpad[1] << 8) | scratchpad[0];

		if(ds18_family == 0x10)
		{
			// DS18S20 or old DS1820
			int16_t dT = 128 * (scratchpad[7] - scratchpad[6]);
			dT /= scratchpad[7];
			raw = 64 * (raw & 0xFFFE) - 32 + dT;
			DS1820_LOG(DEBUG, "family=%x, raw=%i, count_remain=%i, count_per_c=%i, dT=%i", ds18_family, raw, scratchpad[6], scratchpad[7], dT);
		}
		else
		{ // DS18B20
			uint8_t cfg = scratchpad[4] & 0x60;
			if(cfg == 0x00) raw = raw & ~7;      // 9 bit resolution, 93.75 ms
			else if(cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
			else if(cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
			raw = raw << 3; // multiply by 8
			DS1820_LOG(DEBUG, "family=%x, raw=%i, cfg=%x (%i bit resolution)", ds18_family, raw, cfg, 9 + (cfg) / 32);
		}

		// Raw is t * 128
		t = (raw / 128) * 100; // Whole degrees
		int frac = (raw % 128) * 100 / 128; // Fractional degrees
		t += t > 0 ? frac : -frac;

		dsread = 0;
		lastconv = g_secondsElapsed;
		CHANNEL_Set(g_cfg.pins.channels[Pin], t, CHANNEL_SET_FLAG_SILENT);
		DS1820_LOG(INFO, "Temp=%i.%02i", (int)t / 100, (int)t % 100);
		
		return;
	}
	
	if(g_secondsElapsed % ds18_conversionPeriod == 0 || lastconv == 0) //dsread == 0
	{
		if(ds18_family == 0)
		{
			int discovered = DS1820_DiscoverFamily();
			if(!discovered)
			{
				lastconv = -1; // reset lastconv to avoid immediate retry
				DS1820_LOG(ERROR, "Family not discovered");
				return;
			}
		}

		if(OWReset(Pin) == 0)
		{
			lastconv = -1; // reset lastconv to avoid immediate retry
			DS1820_LOG(ERROR, "Reset failed");
#if DS1820_DEBUG
			// if device is not found, maybe "usleep" is not working as expected
			// lets do usleepds() with numbers 50.000 and 100.00
			// if all is well, it should take 50ms and 100ms
			// if not, we need to "calibrate" the loop
			int tempsleep = 5000;
			portTickType actTick = portTICK_RATE_MS * xTaskGetTickCount();
			usleepds(tempsleep);
			int duration = (int)(portTICK_RATE_MS * xTaskGetTickCount() - actTick);

			DS1820_LOG(DEBUG, "usleepds(%i) took %i ms ", tempsleep, duration);

			tempsleep = 100000;
			actTick = portTICK_RATE_MS * xTaskGetTickCount();
			usleepds(tempsleep);
			duration = (int)(portTICK_RATE_MS * xTaskGetTickCount() - actTick);

			DS1820_LOG(DEBUG, "usleepds(%i) took %i ms ", tempsleep, duration);

			if(duration < 95 || duration > 105)
			{
				// calc a new factor for usleepds
				DS1820_LOG(ERROR, "usleepds duration divergates - proposed factor to adjust usleepds %f ", (float)100 / duration);
			}
#endif
			return;
		}

		DS1820_LOG(INFO, "Starting conversion");
		OWWriteByte(Pin, SKIP_ROM);
		OWWriteByte(Pin, CONVERT_T);

		errcount = 0;
		dsread = 1;
	}
}
