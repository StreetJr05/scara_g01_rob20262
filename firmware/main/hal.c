/**
 * @file hal.c
 * @brief Hardware abstraction implementation (ESP-IDF v5.x drivers).
 *
 * MC33926 wiring: IN1/IN2 fixed by hardware (opposite levels), INV swaps the
 * bridge direction, D2 is the inverted disable (low = outputs tri-stated).
 *
 * MOTOR_DRIVE_ANTIPHASE = 1 (default): locked anti-phase. The PWM goes to INV
 * and D2 is held high while the motor is driven, so the bridge alternates
 * forward/reverse every period: V_motor = (1 - 2 d) * Vbat, linear through
 * zero (d = 0.5 -> 0 V, which also brakes/holds). hal_motors_off() pulls D2
 * low -> coast.
 *
 * MOTOR_DRIVE_ANTIPHASE = 0: sign-magnitude drive/coast, PWM on D2 and INV as
 * direction. The off phase tri-states the bridge, the current returns to the
 * supply through the diodes and V_motor ~ (2 d - 1) * Vbat: almost no torque
 * below ~50 % duty (measured: joints break away only at |u| ~ 0.5).
 */
#include "hal.h"
#include "scara_config.h"

#include <math.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"

typedef struct {
    int pin_dir, pin_pwm, pin_enc_a, pin_enc_b, pin_lim_r, pin_lim_l;
    bool motor_invert, enc_invert;
    float counts_per_rev;
    ledc_channel_t ledc_ch;
    pcnt_unit_handle_t pcnt;
    int32_t offset_counts;   /* software zero: pos = raw - offset */
} joint_hw_t;

static joint_hw_t s_hw[NUM_JOINTS] = {
    { J1_PIN_DIR, J1_PIN_PWM, J1_PIN_ENC_A, J1_PIN_ENC_B, J1_PIN_LIM_RIGHT, J1_PIN_LIM_LEFT,
      J1_MOTOR_INVERT, J1_ENC_INVERT, J1_COUNTS_PER_REV, LEDC_CHANNEL_0, NULL, 0 },
    { J2_PIN_DIR, J2_PIN_PWM, J2_PIN_ENC_A, J2_PIN_ENC_B, J2_PIN_LIM_RIGHT, J2_PIN_LIM_LEFT,
      J2_MOTOR_INVERT, J2_ENC_INVERT, J2_COUNTS_PER_REV, LEDC_CHANNEL_1, NULL, 0 },
};

#define PWM_MAX_COUNT ((1u << PWM_RES_BITS) - 1u)
#define PCNT_LIMIT    30000

static void motors_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)PWM_RES_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int j = 0; j < NUM_JOINTS; j++) {
#if MOTOR_DRIVE_ANTIPHASE
        /* D2 = plain output, low (bridge disabled) until the first command */
        gpio_config_t en = {
            .pin_bit_mask = 1ULL << s_hw[j].pin_pwm,
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&en));
        gpio_set_level(s_hw[j].pin_pwm, 0);

        ledc_channel_config_t ch = {      /* PWM on INV, 50 % = 0 V */
            .gpio_num = s_hw[j].pin_dir,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = s_hw[j].ledc_ch,
            .timer_sel = LEDC_TIMER_0,
            .duty = PWM_MAX_COUNT / 2,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
#else
        gpio_config_t dir = {
            .pin_bit_mask = 1ULL << s_hw[j].pin_dir,
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&dir));
        gpio_set_level(s_hw[j].pin_dir, 0);

        ledc_channel_config_t ch = {
            .gpio_num = s_hw[j].pin_pwm,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = s_hw[j].ledc_ch,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
#endif
    }
}

static void encoders_init(void)
{
    for (int j = 0; j < NUM_JOINTS; j++) {
        pcnt_unit_config_t unit_cfg = {
            .high_limit = PCNT_LIMIT,
            .low_limit = -PCNT_LIMIT,
            .flags.accum_count = 1,   /* extend the 16-bit HW counter in SW */
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_hw[j].pcnt));

        pcnt_glitch_filter_config_t filter = { .max_glitch_ns = ENC_GLITCH_NS };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_hw[j].pcnt, &filter));

        /* Full x4 quadrature decoding with two channels */
        pcnt_chan_config_t a_cfg = { .edge_gpio_num = s_hw[j].pin_enc_a, .level_gpio_num = s_hw[j].pin_enc_b };
        pcnt_chan_config_t b_cfg = { .edge_gpio_num = s_hw[j].pin_enc_b, .level_gpio_num = s_hw[j].pin_enc_a };
        pcnt_channel_handle_t ch_a, ch_b;
        ESP_ERROR_CHECK(pcnt_new_channel(s_hw[j].pcnt, &a_cfg, &ch_a));
        ESP_ERROR_CHECK(pcnt_new_channel(s_hw[j].pcnt, &b_cfg, &ch_b));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

        /* accum_count needs watch points at the limits */
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_hw[j].pcnt, PCNT_LIMIT));
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_hw[j].pcnt, -PCNT_LIMIT));

        ESP_ERROR_CHECK(pcnt_unit_enable(s_hw[j].pcnt));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_hw[j].pcnt));
        ESP_ERROR_CHECK(pcnt_unit_start(s_hw[j].pcnt));
    }
}

static void io_init(void)
{
    uint64_t lim_mask = 0;
    for (int j = 0; j < NUM_JOINTS; j++) {
        lim_mask |= (1ULL << s_hw[j].pin_lim_r) | (1ULL << s_hw[j].pin_lim_l);
    }
    gpio_config_t lim = {
        .pin_bit_mask = lim_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&lim));

    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_SOLENOID) | (1ULL << PIN_VALVE),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    hal_valve_set(false);
    hal_solenoid_set(false);
}

void hal_init(void)
{
    io_init();
    motors_init();
    encoders_init();
}

void hal_motor_set(int joint, float duty)
{
    joint_hw_t *hw = &s_hw[joint];
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;
#if MOTOR_DRIVE_ANTIPHASE
    /* fraction of the period with INV high (= reverse): V = (1 - 2 d) Vbat */
    const float u = hw->motor_invert ? -duty : duty;
    const float d = 0.5f * (1.0f - u);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, hw->ledc_ch, (uint32_t)(d * PWM_MAX_COUNT + 0.5f));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, hw->ledc_ch);
    gpio_set_level(hw->pin_pwm, 1);   /* bridge enabled */
#else
    bool reverse = (duty < 0.0f) ^ hw->motor_invert;
    gpio_set_level(hw->pin_dir, reverse ? 1 : 0);
    uint32_t cnt = (uint32_t)(fabsf(duty) * PWM_MAX_COUNT + 0.5f);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, hw->ledc_ch, cnt);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, hw->ledc_ch);
#endif
}

void hal_motors_off(void)
{
    for (int j = 0; j < NUM_JOINTS; j++) {
#if MOTOR_DRIVE_ANTIPHASE
        gpio_set_level(s_hw[j].pin_pwm, 0);   /* D2 low: outputs tri-stated, motor coasts */
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, s_hw[j].ledc_ch, PWM_MAX_COUNT / 2);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, s_hw[j].ledc_ch);
#else
        hal_motor_set(j, 0.0f);
#endif
    }
}

static int32_t encoder_raw(int joint)
{
    int count = 0;
    pcnt_unit_get_count(s_hw[joint].pcnt, &count);
    return s_hw[joint].enc_invert ? -count : count;
}

float hal_encoder_get_rad(int joint)
{
    int32_t c = encoder_raw(joint) - s_hw[joint].offset_counts;
    return (float)c * (2.0f * (float)M_PI) / s_hw[joint].counts_per_rev;
}

void hal_encoder_set_rad(int joint, float rad)
{
    int32_t want = (int32_t)lroundf(rad * s_hw[joint].counts_per_rev / (2.0f * (float)M_PI));
    s_hw[joint].offset_counts = encoder_raw(joint) - want;
}

bool hal_limit_right_raw(int joint) { return gpio_get_level(s_hw[joint].pin_lim_r) == 0; }
bool hal_limit_left_raw(int joint)  { return gpio_get_level(s_hw[joint].pin_lim_l) == 0; }

void hal_solenoid_set(bool on)
{
    gpio_set_level(PIN_SOLENOID, on ? SOLENOID_ACTIVE_LEVEL : !SOLENOID_ACTIVE_LEVEL);
}

void hal_valve_set(bool on)
{
    gpio_set_level(PIN_VALVE, on ? VALVE_ACTIVE_LEVEL : !VALVE_ACTIVE_LEVEL);
}
