# Project Structure

This repository follows an ODrive-style layout: the root directory stays focused on project-level files, while firmware code is grouped under `Firmware/`.

## Root Layout

```text
CMS32FOC_GCC/
├── Firmware/
├── Tools/
├── Docs/
├── Reference/
├── cmake/
├── CMakeLists.txt
├── CMakePresets.json
└── README.md
```

## Firmware Layout

```text
Firmware/
├── App/
├── Board/
├── Drivers/
├── MotorControl/
├── Tests/
└── ThirdParty/
```

- `Firmware/App`: main firmware entry and application-level scheduling.
- `Firmware/Board`: board support package, pin mux, PWM/ADC/MA600, and safety state.
- `Firmware/Board/Config`: project configuration shared by board and control code.
- `Firmware/Drivers/CMS32M6510`: startup, linker script, memory map, and platform glue.
- `Firmware/MotorControl`: lightweight C++ motor-control domain.
- `Firmware/Tests`: flashable diagnostic firmwares.
- `Firmware/ThirdParty`: vendor and external packages; treat as read-only.

## Tools Layout

```text
Tools/
├── JLink/
└── Ozone/
```

- `Tools/JLink`: notes for Windows J-Link flashing; flash commands are generated inline by VS Code tasks.
- `Tools/Ozone`: Ozone project files and path helper scripts.

## Reference Layout

```text
Reference/
├── Hardware/
├── Mechanical/
├── CmsemiconPack/
├── JLink/
├── LXQS_FBL2/
└── Legacy/
```

- `Reference` is copied source material for reading, comparison, and staged migration.
- `Reference` is not included by CMake and must not become an active include/source path.
- See `Docs/Architecture/MigrationPlan.md` for the Keil-to-GCC migration policy.

## Rules

- Do not put firmware implementation files at the repository root.
- Do not migrate legacy Keil files by bulk-copying them into active firmware directories.
- Do not edit vendor files under `Firmware/ThirdParty` unless there is no project-owned alternative.
- Keep chip startup/linker work in `Firmware/Drivers/CMS32M6510`.
- Keep board wiring and safety behavior in `Firmware/Board`.
- Keep FOC and control abstractions in `Firmware/MotorControl`.
- Keep flashable smoke/bring-up programs in `Firmware/Tests`.
- Keep generated files under `build/`.
