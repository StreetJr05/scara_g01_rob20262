/**
 * @file main.c
 * @brief RRP SCARA firmware entry point.
 *
 *  core 1: control task (200 Hz): encoders, limit switches, IK, trajectory,
 *          PID, PWM, solenoid / valve sequencing      -> control.c
 *  core 0: micro-ROS task: topics <-> control interface -> uros_node.c
 *
 * micro-ROS runs over UART0 (the USB cable), so the IDF console is disabled
 * in sdkconfig.defaults (any printf would corrupt the XRCE-DDS stream).
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "driver/uart.h"
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>

#include "esp32_serial_transport.h"
#include "hal.h"
#include "control.h"
#include "uros_node.h"

static size_t uart_port = UART_NUM_0;

void app_main(void)
{
    hal_init();
    control_start();

    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);

    uros_node_start();
}
