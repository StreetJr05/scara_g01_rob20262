/**
 * @file control.h
 * @brief Real-time control task (Ts = CTRL_PERIOD_US) and its thread-safe
 *        interface for the micro-ROS task.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "protocol.h"

void control_start(void);

/* ---- inputs (called from the micro-ROS task) ---- */
void control_post_cmd(int cmd);
void control_post_goto(float x, float y, float z);
void control_set_jog(float j1, float j2);
void control_set_deadman(bool pressed);
void control_set_param(int id, float value);
void control_set_comm(bool connected);

/* ---- outputs ---- */
void control_get_state(float out[ST_LEN]);
/** Pops one pending event string. Returns false if none. */
bool control_pop_event(char *buf, size_t len);
