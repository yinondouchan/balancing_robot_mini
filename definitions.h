#define CYCLE_TIME_MICROS 4096
#define MILLIS_PRESCALED (millis() / 8)
#define MICROS_PRESCALED (micros() / 8)
#define MAIN_LOOP_DIVISOR 0x20

// trimming of balance angle (i.e. manual fine tuning)
typedef void(*trim_callback_t)(int);

// emergency stop callback
typedef void(*estop_callback_t)();
