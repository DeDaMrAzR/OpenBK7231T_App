//////////////////////////////////////////////////////
// specify which parts of the app we wish to be active
//
#ifndef OBK_CONFIG_H
#define OBK_CONFIG_H

// Starts with all driver flags undefined


// NOTE:
// Defines for HTTP/HTMP (UI) pages: ENABLE_HTTP_*
// Defines for drivers from drv_main.c: ENABLE_DRIVER_*
// Other defines: ENABLE_* , for example: ENABLE_LED_BASIC

#define ENABLE_HTTP_MQTT		1
#define ENABLE_HTTP_IP			1
#define ENABLE_HTTP_WEBAPP		1
#define ENABLE_HTTP_NAMES		1
#define ENABLE_HTTP_MAC			1
#define ENABLE_HTTP_FLAGS		1
#define ENABLE_HTTP_STARTUP		1
#define ENABLE_HTTP_PING		1
#define ENABLE_LED_BASIC		1

#if PLATFORM_XR809

#define ENABLE_MQTT 1
#define NO_CHIP_TEMPERATURE			1
#define OBK_DISABLE_ALL_DRIVERS		1

#elif PLATFORM_W600

// parse things like $CH1 or $hour etc
#define ENABLE_EXPAND_CONSTANT	1
#define ENABLE_DRIVER_BMP280 1
#define	ENABLE_HA_DISCOVERY		1
#define ENABLE_DRIVER_AHT2X		1
#define ENABLE_DRIVER_OPENWEATHERMAP	1
#define ENABLE_DRIVER_SSDP		1
#define ENABLE_MQTT 1
#define ENABLE_TASMOTADEVICEGROUPS 1
#define ENABLE_NTP				1
//#define ENABLE_NTP_DST			1
#define ENABLE_DRIVER_BL0937    1
#define ENABLE_DRIVER_DHT		1
#define ENABLE_TASMOTA_JSON		1
#define ENABLE_DRIVER_DS1820		1
#define OBK_OTA_EXTENSION 		".img"
#define ENABLE_OBK_SCRIPTING			1

#elif PLATFORM_W800

// parse things like $CH1 or $hour etc
#define ENABLE_EXPAND_CONSTANT	1
#define	ENABLE_HA_DISCOVERY		1
#define ENABLE_DRIVER_OPENWEATHERMAP	1
#define ENABLE_DRIVER_SSDP		1
#define ENABLE_DRIVER_CHARTS	1
#define ENABLE_MQTT 1
#define ENABLE_DRIVER_SHT3X		1
#define ENABLE_DRIVER_AHT2X		1
#define ENABLE_TASMOTA_JSON		1
#define ENABLE_DRIVER_DS1820		1
#define ENABLE_DRIVER_DHT		1
#define ENABLE_NTP				 1
#define ENABLE_DRIVER_BMP280 1
#define OBK_OTA_EXTENSION 		".img"
#define ENABLE_I2C					1
#define ENABLE_OBK_SCRIPTING			1


#elif WINDOWS


#define	ENABLE_HA_DISCOVERY		1
#define ENABLE_SEND_POSTANDGET		1
#define ENABLE_MQTT 1
#define ENABLE_TASMOTADEVICEGROUPS 1
#define ENABLE_LITTLEFS			1
#define ENABLE_NTP				1
#define ENABLE_NTP_DST			1
#define ENABLE_DRIVER_LED       1
#define ENABLE_DRIVER_BL0937    1
#define ENABLE_DRIVER_BL0942    1
#define ENABLE_DRIVER_BL0942SPI 1
#define ENABLE_DRIVER_CSE7766   1
#define ENABLE_DRIVER_TESTPOWER	1
#define ENABLE_DRIVER_HT16K33   1
#define ENABLE_DRIVER_MAX72XX	1
#define ENABLE_DRIVER_TUYAMCU   1
#define ENABLE_TEST_COMMANDS	1
#define ENABLE_CALENDAR_EVENTS	1
#define ENABLE_TEST_DRIVERS		1
#define ENABLE_DRIVER_BRIDGE	1
#define ENABLE_DRIVER_HTTPBUTTONS		1
#define ENABLE_ADVANCED_CHANNELTYPES_DISCOVERY 1
#define ENABLE_DRIVER_WEMO		1
#define ENABLE_DRIVER_HUE		1
#define ENABLE_DRIVER_CHARGINGLIMIT		1
#define ENABLE_DRIVER_BATTERY	1
#define ENABLE_DRIVER_PT6523	1
#define ENABLE_DRIVER_MAX6675	1
#define ENABLE_DRIVER_TEXTSCROLLER	1
#define ENABLE_NTP_SUNRISE_SUNSET	1
// parse things like $CH1 or $hour etc
#define ENABLE_EXPAND_CONSTANT		1
#define ENABLE_DRIVER_DHT		1
#define	ENABLE_DRIVER_SM16703P	1
#define ENABLE_DRIVER_PIXELANIM	1
#define	ENABLE_DRIVER_TMGN		1
#define ENABLE_DRIVER_DRAWERS	1
#define ENABLE_TASMOTA_JSON		1
#define ENABLE_DRIVER_DDP		1
#define ENABLE_DRIVER_SSDP		1
#define ENABLE_DRIVER_ADCBUTTON	1
#define ENABLE_DRIVER_SM15155E	1
//#define ENABLE_DRIVER_IR		1
//#define ENABLE_DRIVER_IR2		1
#define ENABLE_DRIVER_CHARTS	1
#define ENABLE_DRIVER_WIDGET	1
#define ENABLE_DRIVER_OPENWEATHERMAP	1
#define ENABLE_DRIVER_MCP9808			1
#define ENABLE_DRIVER_KP18058			1
#define ENABLE_DRIVER_ADCSMOOTHER		1
#define ENABLE_DRIVER_SGP				1
#define ENABLE_DRIVER_SHIFTREGISTER		1
#define ENABLE_OBK_SCRIPTING			1

#elif PLATFORM_BL602


#define	ENABLE_HA_DISCOVERY		1
// I have enabled drivers on BL602
#define ENABLE_MQTT 1
#define ENABLE_TASMOTADEVICEGROUPS 1
#define ENABLE_LITTLEFS			1
#define ENABLE_NTP    1
//#define ENABLE_NTP_DST			1
#define ENABLE_CALENDAR_EVENTS	1
#define ENABLE_DRIVER_LED       1
#define ENABLE_DRIVER_BL0937    1
#define ENABLE_DRIVER_BL0942    1
#define ENABLE_DRIVER_CSE7766   1
#define ENABLE_DRIVER_WEMO		1
#define ENABLE_DRIVER_FREEZE	0
#define ENABLE_DRIVER_DHT		1
// parse things like $CH1 or $hour etc
#define ENABLE_EXPAND_CONSTANT	1
#define ENABLE_TASMOTA_JSON		1
#define ENABLE_DRIVER_DDP		1
#define ENABLE_DRIVER_SSDP		1
#define ENABLE_DRIVER_CHT83XX 1
#define ENABLE_DRIVER_DS1820		1
#define ENABLE_OBK_SCRIPTING			1
#define OBK_OTA_EXTENSION 		".bin.xz.ota"
//#define ENABLE_I2C					1


#elif PLATFORM_BEKEN


#define	ENABLE_HA_DISCOVERY		1
#define ENABLE_SEND_POSTANDGET		1
#define ENABLE_MQTT 1
#define ENABLE_TASMOTADEVICEGROUPS 1
#define ENABLE_LITTLEFS			1
#define ENABLE_NTP    1
//#define ENABLE_NTP_DST			1
#define ENABLE_NTP_SUNRISE_SUNSET	1
#define ENABLE_DRIVER_LED       1
#define ENABLE_DRIVER_BL0937    1
#define ENABLE_DRIVER_BL0942    1
#define ENABLE_DRIVER_BL0942SPI 1
#define ENABLE_DRIVER_CSE7766   1
//#define ENABLE_DRIVER_BMP280 1
//#define ENABLE_DRIVER_PT6523	1
//#define ENABLE_DRIVER_MAX6675	1
//#define ENABLE_DRIVER_TEXTSCROLLER	1
#define ENABLE_DRIVER_TUYAMCU   1
//#define ENABLE_DRIVER_HT16K33   1
//#define ENABLE_DRIVER_MAX72XX	  1
//#define ENABLE_DRIVER_ADCBUTTON 1
#define ENABLE_I2C			    1
//#define ENABLE_TEST_COMMANDS	1
#define ENABLE_CALENDAR_EVENTS	1
#define ENABLE_DRIVER_BRIDGE	1
#define ENABLE_DRIVER_HTTPBUTTONS		1
#define ENABLE_ADVANCED_CHANNELTYPES_DISCOVERY 1
#define ENABLE_DRIVER_WEMO		1
#define ENABLE_DRIVER_HUE		1
//#define ENABLE_DRIVER_CHARGINGLIMIT		1
#define ENABLE_DRIVER_BATTERY	1
#if PLATFORM_BK7231N && !PLATFORM_BEKEN_NEW
//#define ENABLE_DRIVER_PWM_GROUP 1
#define ENABLE_DRIVER_SM16703P 1
#define ENABLE_DRIVER_PIXELANIM	1
#define ENABLE_DRIVER_SM15155E 1
#endif
// parse things like $CH1 or $hour etc
#define ENABLE_EXPAND_CONSTANT	1
#define ENABLE_DRIVER_DHT		1
#define ENABLE_DRIVER_AHT2X 1
#define	ENABLE_DRIVER_TMGN		0
#define ENABLE_DRIVER_DRAWERS	0
#define ENABLE_TASMOTA_JSON		1
//#define ENABLE_DRIVER_BMPI2C 1
#define ENABLE_DRIVER_DDP		1
#define ENABLE_DRIVER_SSDP		1
#define ENABLE_DRIVER_IR		1
//#define ENABLE_DRIVER_IR2		1
#define ENABLE_DRIVER_DS1820		1
#define ENABLE_DRIVER_CHT83XX 1
#define ENABLE_DRIVER_KP18058			1
#define ENABLE_DRIVER_ADCSMOOTHER		1
#define ENABLE_OBK_SCRIPTING			1
//#define ENABLE_DRIVER_OPENWEATHERMAP	1
#define OBK_OTA_EXTENSION 		".rbl"
#if PLATFORM_BEKEN_NEW
#define NEW_TCP_SERVER				1
#endif

// ENABLE_I2C_ is a syntax for
// our I2C system defines for drv_i2c_main.c
//#define ENABLE_I2C_ADS1115		1
//#define ENABLE_I2C_MCP23017		1
//#define ENABLE_I2C_LCD_PCF8574	1

#elif PLATFORM_LN882H


#define	ENABLE_HA_DISCOVERY		1
#define ENABLE_MQTT 1
//#define OBK_DISABLE_ALL_DRIVERS       1
#define ENABLE_TASMOTADEVICEGROUPS 1
#define ENABLE_NTP			1
//#define ENABLE_NTP_DST			1
#define ENABLE_DRIVER_BL0937    	1
#define ENABLE_DRIVER_LED 		1
#define ENABLE_DRIVER_WEMO		1
#define ENABLE_DRIVER_HUE		1
#define ENABLE_DRIVER_DHT		1
#define ENABLE_LITTLEFS			1
#define ENABLE_TEST_COMMANDS		0
#define ENABLE_EXPAND_CONSTANT		1
#define ENABLE_DRIVER_OPENWEATHERMAP	1
//#define	ENABLE_DRIVER_TMGN		1
#define ENABLE_TASMOTA_JSON		1
#define ENABLE_DRIVER_DS1820		1
#define ENABLE_OBK_SCRIPTING			1
#define ENABLE_DRIVER_SSDP			1
#define OBK_OTA_EXTENSION 		".bin"
#define OBK_OTA_NAME_EXTENSION 		"_OTA"

#elif PLATFORM_ESPIDF


#define	ENABLE_HA_DISCOVERY		1
#define ENABLE_MQTT 1
#define ENABLE_I2C					1
#define ENABLE_NTP					1
//#define ENABLE_NTP_DST			1
#define ENABLE_DRIVER_LED			1
#define ENABLE_DRIVER_TUYAMCU		1
#define ENABLE_LITTLEFS				1
#define ENABLE_DRIVER_BMPI2C		1
#define ENABLE_DRIVER_DS1820		1
#define ENABLE_DRIVER_DHT			1
#define ENABLE_DRIVER_AHT2X			1
#define ENABLE_DRIVER_BATTERY		1
#define ENABLE_DRIVER_CHARTS		1
#define ENABLE_EXPAND_CONSTANT		1
#define ENABLE_DRIVER_HUE			1
#define ENABLE_DRIVER_WEMO			1
#define ENABLE_DRIVER_BL0937		1
#define ENABLE_TASMOTADEVICEGROUPS	1
#define ENABLE_TASMOTA_JSON			1
#define ENABLE_CALENDAR_EVENTS		1
#define ENABLE_DRIVER_DDP			1
#define ENABLE_DRIVER_SSDP			1
#define ENABLE_DRIVER_CHT83XX		1
#define ENABLE_OBK_SCRIPTING			1
#define OBK_OTA_EXTENSION 		".img"

#elif PLATFORM_TR6260


#define	ENABLE_HA_DISCOVERY		1
#define ENABLE_MQTT 1
#define NO_CHIP_TEMPERATURE			1
#define ENABLE_LITTLEFS				1
#define NEW_TCP_SERVER				1
#define ENABLE_EXPAND_CONSTANT		1
#define ENABLE_I2C					1
#define ENABLE_DRIVER_AHT2X			1
#define ENABLE_DRIVER_BMPI2C		1
#define ENABLE_DRIVER_DS1820		1
#define ENABLE_DRIVER_LED 			1
#define ENABLE_DRIVER_WEMO			1
#define ENABLE_DRIVER_SSDP			1
#define ENABLE_OBK_SCRIPTING		1

#elif PLATFORM_RTL87X0C


#define	ENABLE_HA_DISCOVERY		1
#define ENABLE_MQTT 1
#define NO_CHIP_TEMPERATURE			1
#define ENABLE_LITTLEFS				1
#define NEW_TCP_SERVER				1
#define ENABLE_DRIVER_TUYAMCU		1
#define ENABLE_TASMOTADEVICEGROUPS	1
#define ENABLE_NTP					1
#define ENABLE_CALENDAR_EVENTS		1
#define ENABLE_EXPAND_CONSTANT		1
#define ENABLE_TASMOTA_JSON			1
#define ENABLE_I2C					1
#define ENABLE_DRIVER_AHT2X			1
#define ENABLE_DRIVER_BMPI2C		1
#define ENABLE_DRIVER_DS1820		1
#define ENABLE_DRIVER_LED 			1
#define ENABLE_DRIVER_WEMO			1
#define ENABLE_DRIVER_CHT83XX		1
#define ENABLE_DRIVER_DHT			1
#define ENABLE_DRIVER_BL0942		1
#define ENABLE_DRIVER_BL0937		1
#define ENABLE_DRIVER_CSE7766		1
#define ENABLE_ADVANCED_CHANNELTYPES_DISCOVERY			1
#define OBK_OTA_EXTENSION 			".img"
#define ENABLE_OBK_SCRIPTING		1

#elif PLATFORM_RTL8710B || PLATFORM_RTL8710A

#define	ENABLE_HA_DISCOVERY			1
#define ENABLE_MQTT					1
#define NO_CHIP_TEMPERATURE			1
#define ENABLE_LITTLEFS				1
#define NEW_TCP_SERVER				1
#define ENABLE_DRIVER_TUYAMCU		1
#define ENABLE_TASMOTADEVICEGROUPS	1
#define ENABLE_NTP					1
#define ENABLE_CALENDAR_EVENTS		1
#define ENABLE_EXPAND_CONSTANT		1
#define ENABLE_TASMOTA_JSON			1
#define ENABLE_I2C					1
#define ENABLE_DRIVER_AHT2X			1
#define ENABLE_DRIVER_BMPI2C		1
#define ENABLE_DRIVER_DS1820		1
#define ENABLE_DRIVER_LED 			1
#define ENABLE_DRIVER_WEMO			1
#define ENABLE_DRIVER_CHT83XX		1
#define ENABLE_DRIVER_DHT			1
#define ENABLE_DRIVER_BL0942		1
#define ENABLE_DRIVER_BL0937		1
#define ENABLE_DRIVER_CSE7766		1
#define ENABLE_DRIVER_UART_TCP		1
#define OBK_OTA_EXTENSION 			".img"
#define ENABLE_OBK_SCRIPTING		1

#else

//#error "Platform not defined"
#warning "Platform not defined"

#endif

// if Tasmota DGR driver is enabled,
// then also enable HTTP config page for that
#if ENABLE_TASMOTADEVICEGROUPS
#define ENABLE_HTTP_DGR			1
#endif

// if power metering chip is enabled, also enable backend for that
#if ENABLE_DRIVER_BL0937 || ENABLE_DRIVER_BL0942 || ENABLE_DRIVER_BL0942SPI || ENABLE_DRIVER_CSE7766
#define ENABLE_BL_SHARED	1
#endif

// closing OBK_CONFIG_H
#endif

