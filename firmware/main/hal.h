/**
 * @file hal.h
 * @brief Hardware abstraction: MC33926 motors (LEDC), quadrature encoders
 *        (PCNT), limit switches, solenoid (P axis) and electrovalve.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define NUM_JOINTS 2

void  hal_init(void);

/** duty in [-1, 1]; sign = direction (already corrected by Jx_MOTOR_INVERT). */
void  hal_motor_set(int joint, float duty);
void  hal_motors_off(void);

/** Joint angle in rad (encoder count * scale, corrected by Jx_ENC_INVERT). */
float hal_encoder_get_rad(int joint);
/** Redefine the current encoder position as `rad` (used by homing). */
void  hal_encoder_set_rad(int joint, float rad);

/** Raw (not debounced) limit switch state. true = pressed. */
bool  hal_limit_right_raw(int joint);
bool  hal_limit_left_raw(int joint);

/** on = true energizes the solenoid -> P goes UP. */
void  hal_solenoid_set(bool on);
void  hal_valve_set(bool on);
