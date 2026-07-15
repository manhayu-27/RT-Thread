# Repository Guidelines

## Project Structure & Module Organization

This workspace contains three cooperating components:

- `Demo/`: STM32F407 firmware using RT-Thread and STM32 HAL. CubeMX-generated code is under `Core/`; project-owned acquisition, filtering, and streaming code belongs in `Mycode/`. Build with `MDK-ARM/Demo.uvprojx`; treat `Demo.ioc` as the peripheral configuration source of truth.
- `ESP32/`: ESP-IDF firmware for the ESP32-S3 display and STM32 UART link. Application code is in `main/`, with board assignments centralized in `main/board_pins.h`.
- `web-monitor/`: dependency-free Python HTTP/API server and static browser UI. Read `PROJECT_SPEC.md` before changing its data model or WebSocket protocol. Runtime captures are written to `web-monitor/data/`.

Generated Keil objects, logs, captures, and `analysis_outputs/` are artifacts, not source.

## Build, Test, and Development Commands

Run commands from the relevant module:

```powershell
cd ESP32
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

Use the first command only when configuring a fresh ESP-IDF build. Replace `COMx` with the board port.

```powershell
& "C:\Keil_v5\UV4\UV4.exe" -b "Demo\MDK-ARM\Demo.uvprojx"
python web-monitor\server.py
python -m py_compile web-monitor\server.py
```

The Keil command builds STM32 firmware; the server runs at `http://127.0.0.1:8080`; `py_compile` is the minimum server smoke check.

## Coding Style & Naming Conventions

Follow existing local style. C uses 4-space indentation, `snake_case` functions and variables, uppercase macros, explicit-width integer types, and `U` suffixes for unsigned constants. Keep CubeMX edits inside `USER CODE` blocks; place substantial custom logic in `Demo/Mycode/`. Python follows PEP 8 with type hints where useful. JavaScript uses `camelCase` and two-space indentation. No formatter or linter is configured, so avoid unrelated reformatting.

## Testing Guidelines

There is no automated test suite or coverage gate. Before submission, build each changed firmware target, run `py_compile` for server changes, and exercise affected hardware paths. For UART/protocol work, verify real serial output and keep STM32/ESP32 baud rates and message formats synchronized.

## Commit & Pull Request Guidelines

`Demo/` and `ESP32/` are separate Git repositories; commit in the repository you changed. Existing history uses short, outcome-focused Chinese subjects (for example, `实现lcd显示`). Keep commits scoped to one component and exclude generated binaries and local IDE state. Pull requests should state affected hardware, configuration changes, commands run, and observed results; include UI screenshots or serial excerpts when behavior is visible.
