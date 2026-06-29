# Keil to GCC Migration Plan

This project is the GCC/CMake continuation of the Keil project:

```text
D:\wjw\1_Motor\CMS32M65xx\CMS32FOCAC6
```

The imported reference material is stored under `Reference/`. It is not part of
the CMake build. Treat it as source material for review, comparison, and staged
migration.

## Reference Layout

```text
Reference/
├── Hardware/
├── Mechanical/
├── CmsemiconPack/
├── JLink/
├── LXQS_FBL2/
└── Legacy/CMS32FOCAC6/
```

- `Reference/Hardware`: schematic, CMS32M65xx datasheet/user manual, MA600
  datasheet, and power-chip datasheet.
- `Reference/Mechanical`: actuator outline PDFs and mechanical models.
- `Reference/CmsemiconPack`: CMS32M65xx pack files, SVD, FLM flash loaders, and
  vendor examples.
- `Reference/JLink`: J-Link/J-Scope reference files.
- `Reference/LXQS_FBL2`: LXQS reference firmware/materials for comparison only.
- `Reference/Legacy/CMS32FOCAC6`: original Keil `Docs`, `User` code, project
  files, SVD, and SEGGER RTT files.

Generated Keil output directories such as `Objects` and `Listings` are excluded.

## Migration Boundaries

Keep the GCC project split aligned with the ODrive-style layout:

```text
Firmware/App          -> application entry and scheduling only
Firmware/Board        -> board wiring, pin mux, PWM, ADC/PGA, MA600, safety IO
Firmware/MotorControl -> C++ control-domain objects and FOC/control algorithms
Firmware/Drivers      -> CMS32M6510 startup, linker, memory map, platform glue
Firmware/ThirdParty   -> vendor package; read-only unless unavoidable
Firmware/Tests        -> flashable bring-up and diagnostic images
```

Do not copy legacy Keil files directly into the active firmware tree. Migrate
one behavior at a time, preserving the GCC layering and C/C++ boundary.

## Legacy Code Mapping

```text
Reference/Legacy/CMS32FOCAC6/User/App/Src/main.c
  -> Firmware/App/main.c
     Keep this thin. Call board init and MotorControl entry points.

Reference/Legacy/CMS32FOCAC6/User/App/Src/App_Debug.c
Reference/Legacy/CMS32FOCAC6/User/App/Inc/App_Debug.h
  -> Firmware/Tests/* or a future project-owned debug/watch module.
     Prefer explicit global watch structs for Ozone.

Reference/Legacy/CMS32FOCAC6/User/Board/Src/Board.c
Reference/Legacy/CMS32FOCAC6/User/Board/Src/Board_PWM.c
Reference/Legacy/CMS32FOCAC6/User/Board/Src/Board_Analog.c
Reference/Legacy/CMS32FOCAC6/User/Board/Src/Board_MA600.c
  -> Firmware/Board/Src/foc_bsp.c
  -> Firmware/Board/Src/foc_pwm.c
  -> Firmware/Board/Src/foc_curr.c
  -> Firmware/Board/Src/foc_ma600.c
     Keep register/vendor-driver details here. Expose small C APIs upward.

Reference/Legacy/CMS32FOCAC6/User/Config/Config.h
  -> Firmware/Board/Config/Config.h
     Board constants, motor constants, pin assignments, and safety defaults.

Reference/Legacy/CMS32FOCAC6/User/Motor/Src/Motor*.c
Reference/Legacy/CMS32FOCAC6/User/Motor/Inc/Motor*.h
  -> Firmware/MotorControl/Src/*.cpp
  -> Firmware/MotorControl/Inc/*.hpp
     Port algorithms into small C++ classes. Keep IRQ wrappers C ABI.
```

## C/C++ Rules

- C remains the boundary for BSP, vendor drivers, startup, and interrupt symbols.
- C++17 is allowed in `Firmware/MotorControl` only.
- No exceptions, RTTI, iostream, heap allocation, or hidden runtime-heavy STL.
- Motor-control code talks to hardware through `Firmware/Board` APIs, not direct
  vendor registers.
- Interrupt handlers should call short C wrappers that delegate to board/control
  functions.
- Shared debug variables should be explicit, `volatile` where appropriate, and
  easy to watch in Ozone.

## Suggested Migration Order

1. Keep startup/linker/build stable.
2. Validate board safety state: PWM disabled, brake/enable pins safe.
3. Validate MA600 read path.
4. Validate ADC/PGA current sampling and calibration.
5. Validate PWM timing and ADC trigger alignment.
6. Port open-loop motor test.
7. Port zero/electrical-angle scan.
8. Port current-loop math.
9. Port velocity/position loop pieces.
10. Introduce RTT only after the core bring-up path is stable.

Each step should leave `cms32foc` buildable and preferably add or update a
flashable diagnostic under `Firmware/Tests`.
