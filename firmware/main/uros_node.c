/**
 * @file uros_node.c
 * @brief micro-ROS node "scara_esp32": topics described in protocol.h.
 *
 * Runs its own task (core 0). Survives agent restarts / cable unplugging with
 * the usual ping-based reconnection state machine. All robot logic lives in
 * control.c; callbacks only forward data through control_*().
 */
#include "uros_node.h"
#include "control.h"
#include "protocol.h"
#include "scara_config.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

#define RCCHECK(fn) do { if ((fn) != RCL_RET_OK) return false; } while (0)
#define RCSOFTCHECK(fn) do { rcl_ret_t rc_ = (fn); (void)rc_; } while (0)

#define EVENT_BUF_LEN 128

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_timer_t state_timer;

static rcl_subscription_t sub_pose, sub_jog, sub_deadman, sub_cmd, sub_params;
static rcl_publisher_t pub_state, pub_event;

static geometry_msgs__msg__Point   pose_msg;
static geometry_msgs__msg__Vector3 jog_msg;
static std_msgs__msg__Bool         deadman_msg;
static std_msgs__msg__Int32        cmd_msg;
static std_msgs__msg__Float32MultiArray params_msg;
static float params_buf[PARAMS_MSG_CAPACITY];

static std_msgs__msg__Float32MultiArray state_msg;
static float state_buf[ST_LEN];
static std_msgs__msg__String event_msg;
static char event_buf[EVENT_BUF_LEN];

/* ---------------- callbacks ---------------- */

static void pose_cb(const void *m)
{
    const geometry_msgs__msg__Point *p = m;
    control_post_goto((float)p->x, (float)p->y, (float)p->z);
}

static void jog_cb(const void *m)
{
    const geometry_msgs__msg__Vector3 *v = m;
    control_set_jog((float)v->x, (float)v->y);
}

static void deadman_cb(const void *m)
{
    control_set_deadman(((const std_msgs__msg__Bool *)m)->data);
}

static void cmd_cb(const void *m)
{
    control_post_cmd(((const std_msgs__msg__Int32 *)m)->data);
}

static void params_cb(const void *m)
{
    const std_msgs__msg__Float32MultiArray *a = m;
    for (size_t i = 0; i + 1 < a->data.size; i += 2) {
        control_set_param((int)a->data.data[i], a->data.data[i + 1]);
    }
}

static void state_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;
    control_get_state(state_buf);
    RCSOFTCHECK(rcl_publish(&pub_state, &state_msg, NULL));
}

/* ---------------- entities ---------------- */

static void init_messages(void)
{
    memset(&params_msg, 0, sizeof(params_msg));
    params_msg.data.data = params_buf;
    params_msg.data.capacity = PARAMS_MSG_CAPACITY;

    memset(&state_msg, 0, sizeof(state_msg));
    state_msg.data.data = state_buf;
    state_msg.data.capacity = ST_LEN;
    state_msg.data.size = ST_LEN;

    event_msg.data.data = event_buf;
    event_msg.data.capacity = EVENT_BUF_LEN;
    event_msg.data.size = 0;
}

static bool create_entities(void)
{
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "scara_esp32", "", &support));

    /* best effort for high-rate streams, reliable for one-shot commands */
    RCCHECK(rclc_subscription_init_default(&sub_pose, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), TOPIC_CMD_POSE));
    RCCHECK(rclc_subscription_init_best_effort(&sub_jog, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), TOPIC_JOG));
    RCCHECK(rclc_subscription_init_best_effort(&sub_deadman, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_DEADMAN));
    RCCHECK(rclc_subscription_init_default(&sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), TOPIC_CMD));
    RCCHECK(rclc_subscription_init_default(&sub_params, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), TOPIC_PARAMS));

    RCCHECK(rclc_publisher_init_best_effort(&pub_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), TOPIC_STATE));
    RCCHECK(rclc_publisher_init_default(&pub_event, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TOPIC_EVENT));

    RCCHECK(rclc_timer_init_default2(&state_timer, &support,
        RCL_MS_TO_NS(STATE_PUB_PERIOD_MS), state_timer_cb, true));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_pose, &pose_msg, pose_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_jog, &jog_msg, jog_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_deadman, &deadman_msg, deadman_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, cmd_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_params, &params_msg, params_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &state_timer));
    return true;
}

static void destroy_entities(void)
{
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

    RCSOFTCHECK(rcl_subscription_fini(&sub_pose, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_jog, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_deadman, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_cmd, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_params, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_state, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_event, &node));
    RCSOFTCHECK(rcl_timer_fini(&state_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

static void publish_events(void)
{
    while (control_pop_event(event_buf, EVENT_BUF_LEN)) {
        event_msg.data.size = strlen(event_buf);
        RCSOFTCHECK(rcl_publish(&pub_event, &event_msg, NULL));
    }
}

/* ---------------- task ---------------- */

static void uros_task(void *arg)
{
    enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state = WAITING_AGENT;
    int64_t last_ping = 0;

    init_messages();

    for (;;) {
        switch (state) {
        case WAITING_AGENT:
            state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
            if (state == WAITING_AGENT) vTaskDelay(pdMS_TO_TICKS(400));
            break;
        case AGENT_AVAILABLE:
            if (create_entities()) {
                state = AGENT_CONNECTED;
                control_set_comm(true);
                last_ping = esp_timer_get_time();
            } else {
                destroy_entities();
                state = WAITING_AGENT;
            }
            break;
        case AGENT_CONNECTED:
            if (esp_timer_get_time() - last_ping > 1000000) {
                last_ping = esp_timer_get_time();
                if (rmw_uros_ping_agent(50, 3) != RMW_RET_OK) {
                    state = AGENT_DISCONNECTED;
                    break;
                }
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
            publish_events();
            vTaskDelay(pdMS_TO_TICKS(2));
            break;
        case AGENT_DISCONNECTED:
            control_set_comm(false);
            control_set_deadman(false);
            destroy_entities();
            state = WAITING_AGENT;
            break;
        }
    }
}

void uros_node_start(void)
{
    xTaskCreatePinnedToCore(uros_task, "uros", CONFIG_MICRO_ROS_APP_STACK, NULL,
                            CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL, 0);
}
