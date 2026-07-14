# J-Link Flashing

This project does not keep `.jlink` command files in the repository.

VS Code's `Flash` task opens a firmware picker, generates the J-Link command stream inline, and pipes it directly into Windows `JLink.exe` from WSL.

## Build in WSL

```sh
cd /home/jove/workspace/01-Projects/Embedded/CMS32FOC_GCC
cmake --preset gcc-debug
cmake --build --preset gcc-debug --target cms32foc
```

Diagnostic firmwares can still be built explicitly:

```sh
cmake --build --preset gcc-debug --target cms32_board_watch_test
```

## Flash from VS Code

Run:

```text
Terminal -> Run Task... -> Flash
```

Then choose one target:

```text
cms32foc
cms32_board_watch_test
```

## Manual PowerShell Example

For one-off manual flashing, paste commands into J-Link Commander instead of committing `.jlink` files.

Example target HEX path:

```text
\\wsl.localhost\Ubuntu\home\jove\workspace\01-Projects\Embedded\CMS32FOC_GCC\build\gcc-debug\cms32foc.hex
```

## Debug in Ozone

After flashing, open Ozone and load only the matching ELF for symbols:

```text
\\wsl.localhost\Ubuntu\home\jove\workspace\01-Projects\Embedded\CMS32FOC_GCC\build\gcc-debug\cms32foc
\\wsl.localhost\Ubuntu\home\jove\workspace\01-Projects\Embedded\CMS32FOC_GCC\build\gcc-debug\cms32_board_watch_test
```
