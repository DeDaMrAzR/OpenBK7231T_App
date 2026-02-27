# Rules

## Session Mode
- MODE: STRICT
- WORKDIR: E:\OBK\OBK 
- ACTIVE_TEST_PLATFORM: ESP32-C3
- ACTIVE_CONTROL_PORT: COM7 @ 115200
- No side paths, no extra proposals unless 100% necessary , no unstated actions. 

## Execution Gating
- Plan first when requested.
- Run one approved step at a time.
- Stop after each step and wait for next instruction when asked.

## Command Discipline
- Before execution, provide exact command, target, and expected result.
- Execute only after explicit go-ahead when requested.
- No parallel risky operations on device flash/update paths.

## Logging Discipline
- Log every executed step in the relevant MD file. 
- Each log entry must include: command, result, pass/fail, timestamp.
- Do not continue to next step until log entry is written.

## Failure Policy
- On first error: stop immediately.
- No retries, fallbacks, or alternate methods without approval.


## Completion Checklist
- Report files changed.
- Report tests/logs written under tests if applicable.
