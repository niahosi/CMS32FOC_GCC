# Windows Ozone Debugging

This project builds firmware on Linux/WSL and debugs the resulting ELF from
Windows Ozone.

## Build in Linux/WSL

```sh
cd /home/jove/workspace/01-Projects/Embedded/CMS32FOC_GCC
cmake --preset gcc-debug
cmake --build --preset gcc-debug --target cms32_board_watch_test
```

The ELF loaded by Ozone is:

```text
\\wsl.localhost\Ubuntu\home\jove\workspace\01-Projects\Embedded\CMS32FOC_GCC\build\gcc-debug\cms32_board_watch_test
```

## Debug in Windows Ozone

Open:

```text
Tools\Ozone\cms32_board_watch_test.jdebug
```

Expected settings:

- Device: `Cortex-M0+`
- Interface: `SWD`
- Speed: `4 MHz`
- Program file: the WSL UNC ELF path above

This Ozone install does not know `CMS32M6510` as a device name, so the project
uses the generic Cortex-M0+ core and the generic Cortex-M0 SVD.

If Ozone cannot open source files, add this path substitution in Ozone:

```text
/home/jove/workspace/01-Projects/Embedded/CMS32FOC_GCC
  -> \\wsl.localhost\Ubuntu\home\jove\workspace\01-Projects\Embedded\CMS32FOC_GCC
```

## Watch Variables

Watch the global variable:

```text
g_board_watch
```

Useful fields:

- `boot_magic`: should be `0x32574F46`
- `loop_count`: should increment
- `pwm_off_safe`: should be `1`
- `pwm_output_on`: should be `0`
- `pwm_brake_on`: should be `1`
- `ma600_raw`: should change when the magnet/rotor is moved
- `ma600_ok`: should be `1` when SPI reads are valid
- `adc_irq_count` and `adc_sync_count`: should increment when ADC/PWM sync IRQs run

## Switching to the main firmware

Build:

```sh
cmake --build --preset gcc-debug --target cms32foc
```

Then change the Ozone program/debug file path from:

```text
cms32_board_watch_test
```

to:

```text
cms32foc
```
