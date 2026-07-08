# Control Layer Policy

## Current Rule

`cms32foc` is a pure C bring-up firmware. It links `cms32_motor_control_c` and uses it as the only active motor-control path for VF rotation, current sampling, PWM/ADC timing, and Ozone diagnostics.

The C++ control layer remains in the repository as the final closed-loop architecture reference, but no C++ CMake targets are currently defined. The default project enables only C and ASM.

## Bring-up Path

The active runtime path is intentionally short:

```text
main.c
  -> bsp_init()
  -> MotorControl_Init()
  -> bsp_start_adc_sync()
  -> ADC_IRQHandler
  -> curr_irq()
  -> MotorControl_FastLoopFromAdcIrq()
```

VF mode in the C bring-up path must stay independent from MA600, align, current PI, and the C++ Axis state machine. Its job is to produce a predictable open-loop voltage vector while current sampling is being tuned.

Current-loop and speed-loop bring-up notes are recorded in `Docs/Architecture/CurrentLoopBringupNotes.md`, including electrical zero, sensor direction, q-axis torque, and dynamic `id` behavior.

The current source-of-truth runtime diagram is `Docs/Architecture/ActiveControlChain.md`.

## C++ Recovery Rule

Do not restore the old C++ Axis behavior directly into `cms32foc`.

When the C++ layer is reintroduced, it must inherit the behavior already proven in the C bring-up path:

- no ISR fast-loop dependency on application loop-stage test markers;
- no repeated `pwm_off()` from VF diagnostic state transitions;
- VF diagnostics stay outside the closed-loop Axis state machine;
- Current, Speed, Align, and MA600 checks are restored only after VF sampling is stable.

## Build Boundary

Expected main firmware link boundary:

```text
cms32foc -> cms32_motor_control_c -> cms32_bsp + cms32_foc_algorithm
```

With the default configuration, no C++ targets are created, no `.cpp` files are compiled, and no C++ control object may appear in `cms32foc.map`. To intentionally resume C++ work later, re-enable CXX in `project()` and add back explicit C++ targets.
