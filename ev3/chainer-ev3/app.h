#include "target_test.h"
#define MAIN_PRIORITY    5
#define HIGH_PRIORITY    9
#define MID_PRIORITY    10
#define LOW_PRIORITY    11

#ifndef STACK_SIZE
#define    STACK_SIZE      4096
#endif /* STACK_SIZE */

#ifndef LOOP_REF
#define LOOP_REF        ULONG_C(1000000)
#endif /* LOOP_REF */

#ifndef TOPPERS_MACRO_ONLY

extern void    task(intptr_t exinf);
extern void    main_task(intptr_t exinf);
extern ulong_t  get_time();
extern void     watchdog_task(intptr_t exinf);

typedef enum message_id {
    MOTOR_CONFIG = 0,
    ENABLE_WATCHDOG_TASK = 1,
    MOTOR_STEER = 10,
    SENSOR_CONFIG = 100,
    TOUCH_SENSOR_IS_PRESSED = 110,
    COLOR_SENSOR_GET_REFLECT = 120,
    LCD_DRAW_STRING = 200,
    BUTTON_IS_PRESSED = 210,
    GYRO_SENSOR_RESET = 130,
    GYRO_SENSOR_GET_ANGLE = 131,
    GYRO_SENSOR_GET_RATE = 132,
} msg_id;

#endif /* TOPPERS_MACRO_ONLY */
