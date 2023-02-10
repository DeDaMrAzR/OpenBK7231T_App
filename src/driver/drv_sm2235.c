#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../logging/logging.h"
#include "drv_local.h"
#include "drv_uart.h"
#include "../httpserver/new_http.h"
#include "../hal/hal_pins.h"

#include "drv_sm2235.h"

void SM2235_Write(float *rgbcw) {
	unsigned short cur_col_10[5];
	int i;

	//ADDLOG_DEBUG(LOG_FEATURE_CMD, "Writing to Lamp: %f %f %f %f %f", rgbcw[0], rgbcw[1], rgbcw[2], rgbcw[3], rgbcw[4]);

	for (i = 0; i < 5; i++) {
		// convert 0-255 to 0-1023
		//cur_col_10[i] = rgbcw[g_channelOrder[i]] * 4;
		cur_col_10[i] = MAP(rgbcw[g_channelOrder[i]], 0, 255.0f, 0, 1023.0f);
	}

#define SM2235_FIRST_BYTE(x) ((x >> 8) & 0xFF)
#define SM2235_SECOND_BYTE(x) (x & 0xFF)

	// Byte 0
	Soft_I2C_Start(SM2235_BYTE_0);
	// Byte 1
	Soft_I2C_WriteByte(SM2235_BYTE_1);
	// Byte 2
	Soft_I2C_WriteByte((uint8_t)(SM2235_FIRST_BYTE(cur_col_10[0])));  //Red
	// Byte 3
	Soft_I2C_WriteByte((uint8_t)(SM2235_SECOND_BYTE(cur_col_10[0])));
	// Byte 4
	Soft_I2C_WriteByte((uint8_t)(SM2235_FIRST_BYTE(cur_col_10[1]))); //Green
	// Byte 5
	Soft_I2C_WriteByte((uint8_t)(SM2235_SECOND_BYTE(cur_col_10[1])));
	// Byte 6
	Soft_I2C_WriteByte((uint8_t)(SM2235_FIRST_BYTE(cur_col_10[2]))); //Blue
	// Byte 7
	Soft_I2C_WriteByte((uint8_t)(SM2235_SECOND_BYTE(cur_col_10[2])));
	// Byte 8
	Soft_I2C_WriteByte((uint8_t)(SM2235_FIRST_BYTE(cur_col_10[4]))); //Cold
	// Byte 9
	Soft_I2C_WriteByte((uint8_t)(SM2235_SECOND_BYTE(cur_col_10[4])));
	// Byte 10
	Soft_I2C_WriteByte((uint8_t)(SM2235_FIRST_BYTE(cur_col_10[3]))); //Warm
	// Byte 11
	Soft_I2C_WriteByte((uint8_t)(SM2235_SECOND_BYTE(cur_col_10[3])));
	Soft_I2C_Stop();

}

static commandResult_t SM2235_RGBCW(const void *context, const char *cmd, const char *args, int flags){
	const char *c = args;
	float col[5] = { 0, 0, 0, 0, 0 };
	int ci;
	int val;

	ci = 0;

	// some people prefix colors with #
	if(c[0] == '#')
		c++;
	while (*c){
		char tmp[3];
		int r;
		tmp[0] = *(c++);
		if (!*c)
			break;
		tmp[1] = *(c++);
		tmp[2] = '\0';
		r = sscanf(tmp, "%x", &val);
		if (!r) {
			ADDLOG_ERROR(LOG_FEATURE_CMD, "SM2235_RGBCW no sscanf hex result from %s", tmp);
			break;
		}

		ADDLOG_DEBUG(LOG_FEATURE_CMD, "SM2235_RGBCW found chan %d -> val255 %d (from %s)", ci, val, tmp);

		col[ci] = val;

		// move to next channel.
		ci ++;
		if(ci>=5)
			break;
	}

	SM2235_Write(col);

	return CMD_RES_OK;
}
static void SM2235_SetCurrent(int curValRGB, int curValCW) {
	//g_current_setting_rgb = curValRGB;
	//g_current_setting_cw = curValCW;
}

static commandResult_t SM2235_Current(const void *context, const char *cmd, const char *args, int flags){
	/*int valRGB;
	int valCW;
	Tokenizer_TokenizeString(args,0);

	if(Tokenizer_GetArgsCount()<=1) {
		ADDLOG_DEBUG(LOG_FEATURE_CMD, "SM2235_Current: requires 2 arguments [RGB,CW]. Current value is: %i %i!\n",g_current_setting_rgb,g_current_setting_cw);
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	valRGB = Tokenizer_GetArgInteger(0);
	valCW = Tokenizer_GetArgInteger(1);

	SM2235_SetCurrent(valRGB,valCW);*/
	return CMD_RES_OK;
}

// startDriver SM2235
// SM2235_RGBCW FF00000000
void SM2235_Init() {

	g_i2c_pin_clk = PIN_FindPinIndexForRole(IOR_SM2235_CLK,g_i2c_pin_clk);
	g_i2c_pin_data = PIN_FindPinIndexForRole(IOR_SM2235_DAT,g_i2c_pin_data);

	Soft_I2C_PreInit();


	//cmddetail:{"name":"SM2235_RGBCW","args":"[HexColor]",
	//cmddetail:"descr":"Don't use it. It's for direct access of SM2235 driver. You don't need it because LED driver automatically calls it, so just use led_basecolor_rgb",
	//cmddetail:"fn":"SM2235_RGBCW","file":"driver/drv_sm2235.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("SM2235_RGBCW", "", SM2235_RGBCW, NULL, NULL);
	//cmddetail:{"name":"SM2235_Map","args":"[Ch0][Ch1][Ch2][Ch3][Ch4]",
	//cmddetail:"descr":"Maps the RGBCW values to given indices of SM2235 channels. This is because SM2235 channels order is not the same for some devices. Some devices are using RGBCW order and some are using GBRCW, etc, etc. Example usage: SM2235_Map 0 1 2 3 4",
	//cmddetail:"fn":"SM2235_Map","file":"driver/drv_sm2235.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("SM2235_Map", "", CMD_LEDDriverMap, NULL, NULL);
	//cmddetail:{"name":"SM2235_Current","args":"[Value]",
	//cmddetail:"descr":"Sets the maximum current for LED driver.",
	//cmddetail:"fn":"SM2235_Current","file":"driver/drv_sm2235.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("SM2235_Current", "", SM2235_Current, NULL, NULL);
}

