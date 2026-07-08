# VS Code Tasks

This project keeps the VS Code task list short. Use `Terminal -> Run Task...` for all tasks.

## Tasks

- `Build`: configure CMake if needed, then build the main firmware `cms32foc`.
- `Rebuild`: clean the build output, then build `cms32foc`.
- `Clean`: run the CMake clean target for the `gcc-debug` preset.
- `Flash`: open a picker and flash one selected firmware with J-Link.

`Build` is the default build task, so `Ctrl+Shift+B` runs it.

## Flash Picker

`Flash` prompts for one of:

- `cms32foc`
- `cms32_board_watch_test`
- `cms32_startup_smoke_test`

The selected item is flashed by piping inline J-Link commands to Windows `JLink.exe`; the repository does not keep `.jlink` command files.

## Test Firmware Builds

Test firmwares are still available from the command line:

```sh
cmake --build --preset gcc-debug --target cms32_board_watch_test
cmake --build --preset gcc-debug --target cms32_startup_smoke_test
```

C++ targets are frozen and not defined in the current default CMake project.

## Ozone

Use Ozone to load the matching ELF symbols after flashing:

```text
build/gcc-debug/cms32foc
build/gcc-debug/cms32_board_watch_test
build/gcc-debug/cms32_startup_smoke_test
```
