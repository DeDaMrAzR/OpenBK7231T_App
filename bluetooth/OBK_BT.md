# OBK BK7231N BLE Build Notes

## Purpose
This document records the exact changes used to make `OpenBK7231N_ALT` BLE scanning build and run successfully, with repeatable steps.

## Root Cause Summary
The fix required multiple layers:

1. `OpenBK7231N_ALT` flow previously called `generate_beken_libs.sh` in a way that was brittle for this tree and could produce broken link inputs.
2. BLE 5.x public APIs were used, but `libble_pub.a` was not linked for BK7231N BLE 5.1.
3. BK7231N linker script (`.tcm`) overflowed once BLE 5.1 objects were linked.
4. Docker incremental sync excluded `sdk/`, so stale SDK state could persist across clean builds.

## Touched Files

### Main repo (`e:\OBK\OBK`)
- `Makefile`
- `docker/entrypoint.sh`

### SDK submodule (`e:\OBK\OBK\sdk\beken_freertos_sdk`)
- `tools/scripts/generate_beken_libs.sh`
- `application.mk`
- `build/bk7231n.lds`
- `build/bk7231n_bsp.lds`

## What Changed

### 1) OpenBK7231N_ALT build flow in root Makefile
`OpenBK7231N_ALT` now:
- patches `sys_config_bk7231n.h` for BLE on + BLE 5.1
- builds
- restores original config file

This avoids fragile lib-regeneration behavior in this path.

### 2) Docker CLEAN_BUILD sync behavior
`docker/entrypoint.sh` now performs a full source sync (including `sdk/`) when `CLEAN_BUILD=1`.

This prevents stale SDK files in `openbk_build_data` from overriding local fixes.

### 3) Script compatibility and BLE 5.1 lib-generation flag
In `tools/scripts/generate_beken_libs.sh`:
- `source` -> `.`
- `==` -> `=`
- added optional `OBK_BK7231N_BLE51=1` handling for `CFG_BLE_VERSION=2`

### 4) Linking BLE public library for BLE 5.x
In `application.mk`, added:
- `-lble_pub` when `CFG_BLE_VERSION != BLE_VERSION_4_2`

This resolves BLE public API symbols (`ble_entry`, `ble_set_notice_cb`, `bk_ble_scan_start`, etc.).

### 5) Linker cleanup for BK7231N TCM overflow
In `build/bk7231n.lds` and `build/bk7231n_bsp.lds`:
- removed explicit BLE 5.x object BSS placement from `.tcm`
- left a note that BLE 5.x BSS is moved to normal RAM

This resolves `.tcm` / `.itcm.code` overlap and out-of-region errors.

## Reproducible Build Steps

1. From repo root:
```powershell
cd e:\OBK\OBK
```

2. Ensure submodule contains the SDK changes:
```powershell
git -C sdk/beken_freertos_sdk status --short
```

3. Clear stale Docker build volume (recommended once after these changes):
```powershell
docker volume rm openbk_build_data
```

4. Build using the same Docker invocation path (GUI or script). For direct Docker reproduction:
```powershell
docker run --rm `
  -v E:\OBK\OBK:/app/source:ro `
  -v C:\Users\DeDaMrAz\Desktop\OBK_bt_test:/app/output `
  -v E:\OBK\OBK\temp_obk_config.h:/app/custom_config.h:ro `
  -v openbk_build_data:/app/build `
  -v openbk_rtk_toolchain:/opt/rtk-toolchain `
  -v openbk_espressif_tools:/app/espressif-cache `
  -v openbk_esp8266_tools:/app/esp8266-cache `
  -v openbk_mbedtls_cache:/app/mbedtls-cache `
  -v openbk_csky_w800_cache:/app/csky-w800-cache `
  -v openbk_csky_txw_cache:/app/csky-txw-cache `
  -v openbk_pip_cache:/app/pip-cache `
  -e APP_VERSION=bluetooth_N `
  -e CLEAN_BUILD=1 `
  -e FLASH_SIZE=2MB `
  -e TXW_PACKAGER_MODE=auto `
  -e TZ="Central Europe Standard Time" `
  openbk_builder OpenBK7231N_ALT
```

5. Flash produced artifact and verify runtime logs.

## Runtime Verification Checklist

Expected scan lifecycle:
- `BLE scan stack initialized`
- `BLE passive scan started (...)`
- repeated `BLE adv ...` lines
- on stop command: `BLE passive scan stopped`
- command response `BLEScan 0 ... OK`

Memory sanity:
- temporary free-memory drop during active scan
- free memory recovers after scan stop

## Notes
- `app.c` implicit declaration warnings seen during build were non-fatal in this flow.
- If build behavior appears inconsistent after changes, re-delete `openbk_build_data` and rebuild.
