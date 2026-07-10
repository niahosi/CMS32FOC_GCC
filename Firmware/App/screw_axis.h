#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCREW_HOME_STATE_IDLE 0U
#define SCREW_HOME_STATE_FAST_RETRACT 1U
#define SCREW_HOME_STATE_FAST_BACKOFF 2U
#define SCREW_HOME_STATE_SLOW_RETRACT 3U
#define SCREW_HOME_STATE_FINAL_BACKOFF 4U
#define SCREW_HOME_STATE_DONE 5U
#define SCREW_HOME_STATE_FAULT 6U

#define SCREW_JOG_PHASE_IDLE 0U
#define SCREW_JOG_PHASE_RUNNING 1U
#define SCREW_JOG_PHASE_PAUSE 2U
#define SCREW_JOG_PHASE_FAULT 3U

typedef struct
{
    uint16_t start_seq;
    uint8_t stop;
    int16_t speed_rpm;
    uint16_t run_ms;
    int16_t iq_limit;
    uint16_t repeat_pairs;
} ScrewJogCommand_t;

typedef struct
{
    uint8_t busy;
    uint8_t fault_seen;
    uint8_t phase;
    uint16_t active_seq;
    int16_t active_speed_rpm;
    uint16_t active_run_ms;
    uint16_t active_repeat_pairs;
    uint16_t completed_pairs;
    uint16_t leg_index;
    uint16_t elapsed_ms;
    uint16_t remaining_ms;
} ScrewJogWatch_t;

typedef struct
{
    uint16_t start_seq;
    uint8_t stop;
    int16_t speed_rpm;
    int16_t slow_speed_rpm;
    int16_t backoff_speed_rpm;
    uint16_t timeout_ms;
    int16_t iq_limit;
} ScrewHomeCommand_t;

typedef struct
{
    uint8_t busy;
    uint8_t homed;
    uint8_t fault_seen;
    uint8_t state;
    uint16_t active_seq;
    int16_t active_speed_rpm;
    int16_t active_slow_speed_rpm;
    int16_t active_backoff_speed_rpm;
    uint16_t elapsed_ms;
    uint16_t remaining_ms;
    uint16_t stall_elapsed_ms;
    int32_t zero_encoder_pos;
    int32_t pos_counts;
} ScrewHomeWatch_t;

typedef struct
{
    uint8_t enable;
    uint8_t stop;
    int32_t target_um;
    int16_t max_speed_rpm;
    int16_t kp_rpm_per_mm;
    uint16_t deadband_um;
    int16_t iq_limit;
} ScrewPositionCommand_t;

typedef struct
{
    uint8_t active;
    uint8_t at_target;
    uint8_t not_homed;
    uint8_t fault_seen;
    uint8_t limited;
    int32_t target_um;
    int32_t pos_um;
    int32_t error_um;
    int32_t target_counts;
    int32_t pos_counts;
    int16_t speed_ref_rpm;
} ScrewPositionWatch_t;

extern volatile ScrewJogCommand_t g_screw_jog_cmd;
extern volatile ScrewJogWatch_t g_screw_jog_watch;
extern volatile ScrewHomeCommand_t g_screw_home_cmd;
extern volatile ScrewHomeWatch_t g_screw_home_watch;
extern volatile ScrewPositionCommand_t g_screw_pos_cmd;
extern volatile ScrewPositionWatch_t g_screw_pos_watch;

void ScrewAxis_Init(void);
void ScrewAxis_Run(void);
void ScrewAxis_OnAdcSample(void);

#ifdef __cplusplus
}
#endif
