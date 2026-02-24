# micro-ROS RP2040 helper

Bare-metal micro-ROS publisher example for the Raspberry Pi Pico, built with the Pico SDK.

## Agent CPU usage — why the state machine matters

A common footgun is calling `rmw_uros_ping_agent()` on **every loop iteration**:

```c
// BAD — burns 13-15 % CPU on the agent side
while (true) {
    state = (RCL_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
```

`rclc_executor_spin_some` **does not block** when there is nothing to process — it returns immediately.
The loop therefore runs at full speed and floods the agent with ping/pong messages over serial, which is
what drives high CPU.

### Fix — time-based throttling with `EXECUTE_EVERY_N_MS`

```c
#define EXECUTE_EVERY_N_MS(MS, X) do {                        \
    static volatile int64_t init = -1;                        \
    if (init == -1) { init = uxr_millis(); }                  \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)
```

Use it inside the state machine (identical to the official micro-ROS Arduino reconnection example):

```c
case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
            ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
            ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    break;
```

`uxr_millis()` is a real wall-clock timer, so the ping is **guaranteed** to fire at most once per
interval regardless of how fast the loop body runs.

## Do you need the state machine?

| Scenario | Recommendation |
|---|---|
| Agent always running before Pico boots, manual reboot on failure | Simple blocking ping + flat loop is fine |
| Agent can die/restart independently (e.g. Docker container restart) | **Keep the state machine** — Pico reconnects automatically without reflashing |
| Sensor that must keep running while disconnected (e.g. IMU DMP FIFO) | **Keep the state machine** — sensor `task()` runs outside the `switch` |

## Building

```bash
cd helpers/microros-rp2040
docker compose run --rm build
# output: src/build/pico_micro_ros_example.uf2
```

Copy the `.uf2` to the Pico in BOOTSEL mode (or to `/mnt/d` if using WSL with drive mount).
