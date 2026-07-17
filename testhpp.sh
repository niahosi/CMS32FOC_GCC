arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m0plus -mthumb \
  -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit \
  -I Firmware/MotorControl/Cpp \
  -I Firmware/MotorControl/C \
  -I Firmware/MotorControl/Inc \
  -I Firmware/MotorControl/Algorithm \
  -I Firmware/Support \
  -I Firmware/Board/Config \
  -I Firmware/Board/Inc \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/CMSIS/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Device/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Driver/inc \
  -x c++ -c -o /tmp/cms32_motor_shell_headers.o - <<'CPP'
#include "motor_control_types.hpp"
#include "motor_control_config.hpp"
#include "motor_command_sanitizer.hpp"

int main()
{
    return 0;
}
CPP

echo "hpp check passed"