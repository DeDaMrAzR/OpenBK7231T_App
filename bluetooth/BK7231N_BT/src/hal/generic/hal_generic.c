#include "../hal_generic.h"
#include "../../logging/logging.h"

void __attribute__((weak)) HAL_RebootModule()
{

}

void __attribute__((weak)) HAL_Delay_us(int delay)
{
	for(volatile int i = 0; i < delay; i++)
	{
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
		__asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
	}
}

void __attribute__((weak)) HAL_Configure_WDT()
{

}

void __attribute__((weak)) HAL_Run_WDT()
{

}

void __attribute__((weak)) HAL_RegisterPlatformSpecificCommands()
{

}

void __attribute__((weak)) HAL_BTScan_SetEnabled(int bEnabled)
{
	(void)bEnabled;
	ADDLOG_INFO(LOG_FEATURE_MAIN, "BLE scan weak HAL active (platform-specific backend not linked)");
}

int __attribute__((weak)) HAL_BTScan_GetEnabled()
{
	return 0;
}
