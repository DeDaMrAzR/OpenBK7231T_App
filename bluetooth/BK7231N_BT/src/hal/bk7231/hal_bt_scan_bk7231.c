#include "../../new_common.h"
#include "../../logging/logging.h"
#include "../hal_generic.h"

#if ENABLE_MQTT
#include "../../mqtt/new_mqtt.h"
#endif

#if PLATFORM_BK7231N && PLATFORM_BEKEN_NEW
#include "include.h"
#if defined(CFG_SUPPORT_BLE) && (CFG_SUPPORT_BLE == 1)
#include "ble_api.h"
#define OBK_BK7231N_BLE_API_AVAILABLE 1
#else
#define OBK_BK7231N_BLE_API_AVAILABLE 0
#endif
#else
#define OBK_BK7231N_BLE_API_AVAILABLE 0
#endif

#define LOG_FEATURE LOG_FEATURE_MAIN

#if defined(CFG_SUPPORT_BLE)
#define OBK_CFG_SUPPORT_BLE_VALUE CFG_SUPPORT_BLE
#else
#define OBK_CFG_SUPPORT_BLE_VALUE -1
#endif

#if OBK_BK7231N_BLE_API_AVAILABLE
static int g_bleScanEnabled = 0;
static int g_bleStackReady = 0;
static uint8_t g_bleScanActvIdx = 0xFF;
static uint16_t g_bleScanInterval = 100;
static uint16_t g_bleScanWindow = 30;

static void HAL_BTScan_CommandCb(ble_cmd_t cmd, ble_cmd_param_t *param) {
	if (param == 0) {
		return;
	}
	ADDLOG_DEBUG(LOG_FEATURE, "BLE cmd %d idx %d status %d", cmd, param->cmd_idx, param->status);
}

static void HAL_BTScan_OnAdv(recv_adv_t *adv) {
	char dataHex[129];
	char mqttJson[220];
	int i;
	int at;
	int dataLen;
	int rssi;

	if (adv == 0) {
		return;
	}

	// keep logs compact on constrained platforms
	dataLen = adv->data_len;
	if (dataLen > 32) {
		dataLen = 32;
	}

	at = 0;
	for (i = 0; i < dataLen && (at + 2) < (int)sizeof(dataHex); i++) {
		at += snprintf(dataHex + at, sizeof(dataHex) - at, "%02X", adv->data[i]);
	}
	dataHex[at] = 0;

	rssi = (int)(int8_t)adv->rssi;
	ADDLOG_INFO(LOG_FEATURE, "BLE adv %02X:%02X:%02X:%02X:%02X:%02X rssi %d evt %u len %u data %s",
		adv->adv_addr[0], adv->adv_addr[1], adv->adv_addr[2],
		adv->adv_addr[3], adv->adv_addr[4], adv->adv_addr[5],
		rssi, adv->evt_type, adv->data_len, dataHex);

#if ENABLE_MQTT
	snprintf(mqttJson, sizeof(mqttJson),
		"{\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"rssi\":%d,\"evt\":%u,\"len\":%u,\"data\":\"%s\"}",
		adv->adv_addr[0], adv->adv_addr[1], adv->adv_addr[2],
		adv->adv_addr[3], adv->adv_addr[4], adv->adv_addr[5],
		rssi, adv->evt_type, adv->data_len, dataHex);
	MQTT_PublishMain_StringString("ble/adv", mqttJson, OBK_PUBLISH_FLAG_FORCE_REMOVE_GET);
#endif
}

static void HAL_BTScan_NoticeCb(ble_notice_t notice, void *param) {
	if (notice == BLE_5_REPORT_ADV && g_bleScanEnabled) {
		HAL_BTScan_OnAdv((recv_adv_t *)param);
	}
}

static void HAL_BTScan_EnsureStackReady() {
	if (g_bleStackReady) {
		return;
	}

	extern void ble_entry(void);
	ble_set_notice_cb(HAL_BTScan_NoticeCb);
	ble_entry();
	g_bleStackReady = 1;
	ADDLOG_INFO(LOG_FEATURE, "BLE scan stack initialized");
}

static void HAL_BTScan_Start() {
	struct scan_param scan;
	ble_err_t ret;

	if (g_bleScanEnabled) {
		return;
	}

	HAL_BTScan_EnsureStackReady();

	memset(&scan, 0, sizeof(scan));
	scan.channel_map = 7;
	scan.interval = g_bleScanInterval;
	scan.window = g_bleScanWindow;

	g_bleScanActvIdx = app_ble_get_idle_actv_idx_handle(SCAN_ACTV);
	if (g_bleScanActvIdx == 0xFF) {
		ADDLOG_ERROR(LOG_FEATURE, "BLE scan failed - no idle activity");
		return;
	}

	ret = bk_ble_scan_start(g_bleScanActvIdx, &scan, HAL_BTScan_CommandCb);
	if (ret != ERR_SUCCESS) {
		ADDLOG_ERROR(LOG_FEATURE, "BLE scan start failed (err %d)", ret);
		g_bleScanActvIdx = 0xFF;
		return;
	}
	g_bleScanEnabled = 1;
	ADDLOG_INFO(LOG_FEATURE, "BLE passive scan started (idx=%u, intv=%u, win=%u)",
		g_bleScanActvIdx, g_bleScanInterval, g_bleScanWindow);
}

static void HAL_BTScan_Stop() {
	ble_err_t ret;

	if (!g_bleScanEnabled) {
		return;
	}

	ret = bk_ble_scan_stop(g_bleScanActvIdx, HAL_BTScan_CommandCb);
	if (ret != ERR_SUCCESS) {
		ADDLOG_ERROR(LOG_FEATURE, "BLE scan stop failed (err %d)", ret);
		return;
	}
	g_bleScanEnabled = 0;
	g_bleScanActvIdx = 0xFF;
	ADDLOG_INFO(LOG_FEATURE, "BLE passive scan stopped");
}
#endif

void HAL_BTScan_SetEnabled(int bEnabled) {
#if OBK_BK7231N_BLE_API_AVAILABLE
	if (bEnabled) {
		HAL_BTScan_Start();
	}
	else {
		HAL_BTScan_Stop();
	}
#else
	(void)bEnabled;
	ADDLOG_INFO(LOG_FEATURE, "BLE scan is not supported on this build (PLATFORM_BEKEN_NEW=%d CFG_SUPPORT_BLE=%d)",
		(int)PLATFORM_BEKEN_NEW, (int)OBK_CFG_SUPPORT_BLE_VALUE);
#endif
}

int HAL_BTScan_GetEnabled() {
#if OBK_BK7231N_BLE_API_AVAILABLE
	return g_bleScanEnabled;
#else
	return 0;
#endif
}
