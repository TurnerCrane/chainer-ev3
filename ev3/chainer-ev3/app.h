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
    // タスク制御等
    ENABLE_WATCHDOG_TASK = 10,
    // モーター
    MOTOR_CONFIG = 50,
    MOTOR_STEER = 51,
    MOTOR_GET_COUNTS = 52,
    MOTOTR_GET_POWER = 53,
    MOTOR_RESET_COUNTS = 54,
    MOTOR_ROTATE = 55,
    MOTOR_SET_POWER = 56,
    MOTOR_STOP = 57,
    // センサ共通
    SENSOR_CONFIG = 100,
    // タッチセンサ
    TOUCH_SENSOR_IS_PRESSED = 110,
    // カラーセンサ
    COLOR_SENSOR_GET_REFLECT = 120,
    COLOR_SENSOR_GET_AMBIENT = 121,
    COLOR_SENSOR_GET_COLOR = 122,
    COLOR_SENSOR_GET_RGB_RAW = 123,
    // ジャイロセンサ
    GYRO_SENSOR_RESET = 130,
    GYRO_SENSOR_GET_ANGLE = 131,
    GYRO_SENSOR_GET_RATE = 132,
    // 超音波センサ
    ULTRASONIC_SENSOR_GET_DISTANCE = 140,
    ULTRASONIC_SENSOR_LISTEN = 141,
    // 赤外線センサ
    INFRARED_SENSOR_GET_DISTANCE = 150,
    INFRARED_SENSOR_GET_REMOTE = 151,
    INFRARED_SENSOR_SEEK = 152,
    // EV3本体
    LCD_DRAW_STRING = 200,
    BUTTON_IS_PRESSED = 210
} msg_id;

#endif /* TOPPERS_MACRO_ONLY */
