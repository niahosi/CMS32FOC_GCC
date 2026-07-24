#!/usr/bin/env sh
set -eu

arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m0plus -mthumb \
  -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit \
  -I Firmware/MotorControl/Abi \
  -I Firmware/MotorControl/Config \
  -I Firmware/MotorControl/Core \
  -I Firmware/MotorControl/Math \
  -I Firmware/MotorControl/Types \
  -I Firmware/MotorControl/Algorithm \
  -I Firmware/Support \
  -I Firmware/Board/Config \
  -I Firmware/Board/Inc \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/CMSIS/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Device/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Driver/inc \
  -x c++ -c -o /tmp/cms32_cpp_headers_check.o - <<'CPP'
#include "types.hpp"
#include "debug_state.hpp"
#include "config.hpp"
#include "command_sanitizer.hpp"
#include "encoder_math.hpp"
#include "motor_controller.hpp"

static_assert(cms32::motor::speed_diff_max_delta_raw() == 28398U,
              "unexpected speed spike raw limit");
static_assert(cms32::motor::deadband_delta(1) == 0, "speed delta deadband failed");
static_assert(cms32::motor::deadband_delta(16) == 16, "speed delta edge changed");

int main()
{
    cms32::motor::MotorController* motor = &cms32::motor::g_motor;
    return motor->speedCountsToRpm(0);
}
CPP
echo "hpp check passed"
