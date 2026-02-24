#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

const uint LED_PIN = 16;
const bool IS_RGBW = false;

// ── helpers ──────────────────────────────────────────────

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) |
           ((uint32_t)(g) << 16) |
           (uint32_t)(b);
}

// ── macros ───────────────────────────────────────────────

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; \
                      if (temp_rc != RCL_RET_OK) { return false; } }

#define EXECUTE_EVERY_N_MS(MS, X) do {                        \
    static volatile int64_t init = -1;                        \
    if (init == -1) { init = uxr_millis(); }                  \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// ── globals ──────────────────────────────────────────────

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// ── callbacks ────────────────────────────────────────────

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL) {
        rcl_publish(&publisher, &msg, NULL);
        msg.data++;
    }
}

// ── entity lifecycle ─────────────────────────────────────

bool create_entities()
{
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "pico_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher"));

    RCCHECK(rclc_timer_init_default(
        &timer, &support,
        RCL_MS_TO_NS(1000),
        timer_callback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    return true;
}

void destroy_entities()
{
    rmw_context_t *rmw_context =
        rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&publisher, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ── main ─────────────────────────────────────────────────

int main()
{
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, LED_PIN, 800000, IS_RGBW);

    msg.data = 0;
    state = WAITING_AGENT;

    while (true)
    {
        switch (state) {
            case WAITING_AGENT:
                // Ping at most every 500 ms
                EXECUTE_EVERY_N_MS(500,
                    state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                        ? AGENT_AVAILABLE : WAITING_AGENT;);
                break;

            case AGENT_AVAILABLE:
                state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) {
                    destroy_entities();
                }
                break;

            case AGENT_CONNECTED:
                // Ping at most every 200 ms
                EXECUTE_EVERY_N_MS(200,
                    state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                        ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                }
                break;

            case AGENT_DISCONNECTED:
                destroy_entities();
                state = WAITING_AGENT;
                break;

            default:
                break;
        }

        // LED update (runs at loop speed, but that's fine —
        // it's just a PIO write, no serial traffic)
        if (state == AGENT_CONNECTED) {
            put_pixel(urgb_u32(0, 50, 0));
        } else {
            put_pixel(urgb_u32(50, 0, 0));
        }
    }

    return 0;
}