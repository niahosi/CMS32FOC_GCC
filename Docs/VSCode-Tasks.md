# VS Code Tasks

This project keeps the VS Code task list short. Use `Terminal -> Run Task...` for all tasks.

## Tasks

- `Build`: configure CMake if needed, then build the main firmware `cms32foc`.
- `Rebuild`: clean the build output, then build `cms32foc`.
- `Clean`: run the CMake clean target for the `gcc-debug` preset.
- `Flash`: open a picker and flash one selected firmware with J-Link at SWD 10 MHz.
- `Flash Rescue`: flash with J-Link at SWD 1 MHz for conservative recovery.

`Build` is the default build task, so `Ctrl+Shift+B` runs it.

## Flash Picker

`Flash` prompts for one of:

- `cms32foc`

The selected item is flashed by piping inline J-Link commands to Windows `JLink.exe`; the repository does not keep `.jlink` command files.

C++ is enabled in CMake for new modules, but the current task list only builds the active firmware.

## Ozone

Use Ozone to load the matching ELF symbols after flashing:

```text
build/gcc-debug/cms32foc
```
