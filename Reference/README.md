# Reference Material

This directory stores source material copied from the original Windows/Keil
workspace and related hardware packages.

It is intentionally outside `Firmware/` and is not part of the CMake build.
Use it for reading, comparison, and staged migration only.

## Key Directories

- `Hardware/`: schematic, CMS32M65xx datasheet/user manual, MA600 datasheet, and
  power-chip datasheet.
- `Mechanical/`: actuator outline PDFs and mechanical models.
- `CmsemiconPack/`: CMS32M65xx pack reference, SVD, FLM flash loaders, and vendor
  examples.
- `JLink/`: J-Link/J-Scope reference files.
- `LXQS_FBL2/`: LXQS reference firmware/materials for comparison.
- `Legacy/CMS32FOCAC6/`: original Keil project docs, user code, SEGGER RTT files,
  SVD, and project files.

For the active migration policy, see:

```text
Docs/Architecture/MigrationPlan.md
```
