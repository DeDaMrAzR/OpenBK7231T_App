# BT Implementation Plan (ESP32-C3 first -> OBK BT Proxy)

## Active Test Platform

- Primary platform now: ESP32-C3
- Local board status: attached to this PC, running OBK
- Control/logging: COM7 @ 115200
- RTL8720D path is deferred until after ESP32-C3 bring-up validation

## Goal

Enable BLE proxy functionality in OBK starting on ESP32-C3, then evolve toward ESPHome-like proxy behavior and port to other supported platforms.

## Current Findings

- ESP32-C3 hardware is available now and ready for immediate iteration.
- OBK currently has no ESPHome native API/protobuf implementation.
- Existing BT plan/architecture remains valid; platform execution order changes.

## Architecture Decision

Use a combined model:

- Platform capability gate: compile BT implementation only for platforms with SDK BT support (ESP32-C3 first; RTL8720D later).
- Feature flag gate: add `ENABLE_BT_PROXY` to enable/disable BT proxy in supported builds.
- Driver/module boundary: keep platform-specific implementation in a dedicated module (`drv_bt_proxy_*` or `hal_bt_*`) with a minimal common interface.

Rationale:

- Avoids leaking platform BLE APIs into generic OBK code.
- Prevents accidental BT linkage on unsupported platforms.
- Keeps binary size and risk controlled via feature flag.
- Makes future porting to other platform BT stacks straightforward.

## Phased Plan

### Phase 1: Build Enablement (ESP32-C3)

1. Add OBK feature flag for ESP32-C3 BT proxy enablement.
2. Enable required BT config macros for ESP32-C3 build profile.
3. Ensure BT libs/includes are linked only when flag is enabled.
4. Build-check firmware image size and link stability.

### Phase 2: Runtime Bring-up (ESP32-C3)

1. Initialize BT stack during platform startup.
2. Verify coexistence with Wi-Fi startup and normal OBK networking.
3. Add basic BT health/status logging.

### Phase 3: BLE Scan Ingest

1. Start BLE scan task (active/passive configurable).
2. Capture advertisement events into OBK-owned ring buffer.
3. Expose scan data in OBK logs/JSON endpoint for validation.

### Phase 4: Proxy Core

1. Implement command queue for connect/read/write/notify operations.
2. Add connection slot and timeout management.
3. Add watchdog/recovery behavior for BT task failures.

### Phase 5: Integration Path Decision

1. Option A: ESPHome-compatible protocol bridge (high effort, best HA compatibility).
2. Option B: OBK-native BLE proxy over MQTT/WebSocket (lower effort, custom integration).
3. Decide and implement one path first; keep interfaces modular.

### Phase 6: Hardening (Deferred)

1. Stress test with many BLE devices and frequent advertisements.
2. Validate memory/CPU overhead and long-run stability.
3. Add docs and operator controls (scan interval/window, active mode, limits).

### Phase 7: Platform Port Follow-up (Deferred)

1. Port validated core/proxy interfaces to RTL8720D BT stack.
2. Keep platform glue isolated; reuse common proxy core.
3. Re-run bring-up/hardening checklist on RTL8720D.

## PoC Conclusion & Findings

- **ESPHome Protobuf TCP Server (Option A):** Successfully implemented on ESP32-C3. Home Assistant correctly discovers the proxy, subscribes to advertisements, successfully parses binary device payloads (e.g. ECOFlow fix), and dispatches active connection requests.
- **Platform Fragmentation:** Each platform (ESP-IDF, Realtek, Beken) has its own entirely disparate Bluetooth API stack. Porting the `hal_bt_proxy_*.c` layer requires massive effort for each chip family.
- **Size Constraints:** Implementing the full host + controller BT stack pushes the firmware size significantly higher. It is highly likely to exceed the available flash size on 2MB platforms (like BK7231N/T and older RTLs).
- **Decision:** Given the high cost-to-benefit ratio, the BT Proxy development is paused here as a successful Proof of Concept (PoC) on ESP32-C3. The working firmware will be left running to gather long-term stability data, but we will not actively port to RTL8720D or 2MB Beken chips at this time.

## Risks / Unknowns

- RAM/flash headroom after enabling BT stack.
- Wi-Fi + BT coexistence impact on reliability.
- Scope/complexity if strict ESPHome protocol compatibility is required.

## Immediate Next Steps (Updated)

1. Conclude the PoC phase.
2. Leave ESP32-C3 test unit running in production to monitor stability.
3. Transition focus to `docker/build_tool.py` and GUI enhancements.

## Licensing Strategy

- Prefer protocol-compatibility implementation over code reuse.
- Implement ESPHome-compatible behavior as clean-room wire/protocol compatibility.
- Do not copy ESPHome firmware internals unless we explicitly decide to import third-party code.
- If any third-party code is imported, track license obligations and add proper attribution/NOTICE entries.

### Practical Licensing Recommendations

1. Build OBK BT proxy stack as native OBK code.
2. Keep an adapter layer that matches ESPHome proxy protocol behavior.
3. Avoid copy/paste from ESPHome sources; use protocol behavior as reference.
4. If external code is used, record source, license, and required notices in-repo.
