# CMS32FOC_GCC Hardware Pins

Source of truth for this bring-up step:
`D:\wjw\1_Motor\CMS32M65xx\CMS32FOCAC6` Keil project, whose pin mapping has
already been checked on hardware.

## Board Layer

- `Firmware/Board/Src/foc_bsp.c`: clock setup, board init order, P16 driver enable
  safe state.
- `Firmware/Board/Src/foc_pwm.c`: EPWM output pin mux and PWM safety control.
- `Firmware/Board/Src/foc_ma600.c`: MA600 SPI pin mux and SSP setup.
- `Firmware/Board/Src/foc_curr.c`: ADC/PGA current sampling pins and synchronized
  EPWM ADC trigger setup.
- `Firmware/Board/Config/Config.h`: PWM period, duty limits, deadtime, ADC trigger,
  PGA gain, and current sampling mode constants.

## Pin Mapping

| Function | Pins | Firmware configuration |
| --- | --- | --- |
| Driver enable | P16 | GPIO output, cleared during init, set only by `pwm_enable(1)` |
| EPWM U/NU | P10/P11 | `EPWM0` / `EPWM1` |
| EPWM V/NV | P12/P13 | `EPWM2` / `EPWM3` |
| EPWM W/NW | P14/P15 | `EPWM4` / `EPWM5` |
| MA600 CS | P02 | `SSIO`, manual SSP CS control |
| MA600 SCLK | P03 | `SCK` |
| MA600 MISO | P04 | `MISO` |
| MA600 MOSI | P05 | `MOSI` |
| U current PGA input | P00/P01 | analog input pair for PGA0 |
| V current PGA input | P24/P25 | analog input pair for PGA1 |
| W current PGA input | P26/P27 | analog input pair for PGA2 |
| ADC current channels | PGA0/PGA1/PGA2 outputs | `ADC_CH_0`, `ADC_CH_2`, `ADC_CH_3` |

## Safety Notes

- P06/P07 are intentionally left untouched so SWD stays available during
  development.
- P16 is low at board init and PWM output remains disabled/braked until an
  explicit enable call.
- The first GCC `main()` only calls `bsp_init()` and then idles; Motor control
  logic is not migrated in this step.
