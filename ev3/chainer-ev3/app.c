#include <string.h>

#include "ev3api.h"
#include "app.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

static FILE *serial = NULL;
static int last_updated_time;
static int left_motor_port = -1;
static int right_motor_port = -1;
static int enable_watchdog_task = 0;

/**
 * Get current time in [msec].
 */
ulong_t get_time()
{
    static ulong_t start = -1;
    ulong_t time;
    get_tim(&time); 
    if(start < 0){
        start = time;
    }
    return time - start;
}

/**
 * This is run in the background every 1 seconds.
 * Send the stop command if idle time is more than 2 seconds.
 */
void watchdog_task(intptr_t exinf) {
    if (left_motor_port == -1 || right_motor_port == -1) return;
    if (enable_watchdog_task != 1) return;
    ulong_t now = get_time();
    if (now - last_updated_time > 1000) {
        ev3_motor_steer(left_motor_port, right_motor_port, 0, 0);
    }
}

uint8_t read_byte(FILE* serial) {
    uint8_t hi = fgetc(serial);
    uint8_t lo = fgetc(serial);
    uint8_t ret = hi*16 + lo;
    return ret;
}

void main_task(intptr_t unused) {
    // Draw information
    lcdfont_t font = EV3_FONT_MEDIUM;
    ev3_lcd_set_font(font);
    int32_t fontw, fonth;
    ev3_font_get_size(font, &fontw, &fonth);
    char lcdstr[100];

    // Open serial port
    serial = ev3_serial_open_file(EV3_SERIAL_UART);
    assert(serial != NULL);

    while (1) {
        last_updated_time = get_time();
        // sprintf(lcdstr, "%08d", last_updated_time);
        // ev3_lcd_draw_string(lcdstr, 0, fonth * 5);

        uint8_t header = read_byte(serial);
        // sprintf(lcdstr, "header: %04d", header);
        // ev3_lcd_draw_string(lcdstr, 0, fonth * 6);
        if (header != 255) continue;

        uint8_t cmd_id = (msg_id)read_byte(serial);
        // sprintf(lcdstr, "cmd_id: %04d", cmd_id);
        // ev3_lcd_draw_string(lcdstr, 0, fonth * 7);

        switch((msg_id)cmd_id) {
            // タスク制御等
            case ENABLE_WATCHDOG_TASK:
                enable_watchdog_task = 1;
                break;
            // モーター
            case MOTOR_CONFIG:
                {
                    int motor_port = read_byte(serial);
                    int motor_type = read_byte(serial);
                    ev3_motor_config(motor_port, motor_type);
                }
                break;
            case MOTOR_STEER:
                {
                    left_motor_port = read_byte(serial);
                    right_motor_port = read_byte(serial);
                    int drive = read_byte(serial) - 100;
                    int steer = read_byte(serial) - 100;
                    if (drive > 100) drive=100;
                    if (drive < -100) drive=-100;
                    if (steer > 100) steer=100;
                    if (steer < -100) steer=-100;
                    ev3_motor_steer(left_motor_port, right_motor_port, drive, steer);
                }
                break;
            case MOTOR_GET_COUNTS:
                {
                    uint8_t motor_port = read_byte(serial);
                    uint32_t counts = ev3_motor_get_counts(motor_port);
                    int i;
                    for(i=3; i>0; i--) {
                        fputc((uint8_t)254, serial);
                        fputc((uint8_t)(counts >> (8 * i)), serial);
                    }
                }
                break;
            case MOTOTR_GET_POWER:
                {
                    uint8_t motor_port = read_byte(serial);
                    uint8_t counts = ev3_motor_get_power(motor_port);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)counts , serial);
                }
                break;
            case MOTOR_RESET_COUNTS:
                {
                    uint8_t motor_port = read_byte(serial);
                    ev3_motor_reset_counts(motor_port);
                }
                break;
            case MOTOR_ROTATE:
                {
                    uint8_t motor_port = read_byte(serial);
                    uint8_t degrees = read_byte(serial);
                    uint32_t speed_abs = read_byte(serial);
                    uint8_t blocking = read_byte(serial);
                    ev3_motor_rotate(motor_port, degrees, speed_abs, blocking);
                }
                break;
            case MOTOR_SET_POWER:
                {
                    uint8_t motor_port = read_byte(serial);
                    uint8_t power = read_byte(serial);
                    ev3_motor_set_power(motor_port, power);
                }
                break;
            case MOTOR_STOP:
                {
                    uint8_t motor_port = read_byte(serial);
                    uint8_t breaking = read_byte(serial);
                    ev3_motor_stop(motor_port, breaking);
                }
                break;
            // センサ共通
            case SENSOR_CONFIG:
                {
                    uint8_t sensor_port = read_byte(serial);
                    uint8_t sensor_type = read_byte(serial);
                    ev3_sensor_config(sensor_port, sensor_type);
                }
                break;
            // タッチセンサ
            case TOUCH_SENSOR_IS_PRESSED:
                {
                    uint8_t touch_sensor_port = read_byte(serial);
                    uint8_t touch = ev3_touch_sensor_is_pressed(touch_sensor_port);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)touch, serial);
                }
                break;
            // カラーセンサ
            case COLOR_SENSOR_GET_REFLECT:
                {
                    uint8_t color_sensor_port = read_byte(serial);
                    uint8_t color = ev3_color_sensor_get_reflect(color_sensor_port);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)color, serial);
                }
                break;
            case COLOR_SENSOR_GET_AMBIENT:
                {
                    uint8_t color_sensor_port = read_byte(serial);
                    uint8_t ambient = ev3_color_sensor_get_ambient(color_sensor_port);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)ambient, serial);
                }
                break;
            case COLOR_SENSOR_GET_COLOR:
                {
                    uint8_t color_sensor_port = read_byte(serial);
                    uint8_t color = ev3_color_sensor_get_color(color_sensor_port);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)color, serial);
                }
                break;
            case COLOR_SENSOR_GET_RGB_RAW:
                {
                    uint8_t color_sensor_port = read_byte(serial);
                    rgb_raw_t rgb_raw;
                    uint16_t rgb[3];
                    ev3_color_sensor_get_rgb_raw(color_sensor_port, &rgb_raw);
                    rgb[0] = rgb_raw.r;
                    rgb[1] = rgb_raw.g;
                    rgb[2] = rgb_raw.b;
                    int i;
                    for(i=0; i<3; i++){
                        fputc((uint8_t)254, serial);
                        fputc((uint8_t)(rgb[i] >> 8), serial);
                        fputc((uint8_t)254, serial);
                        fputc((uint8_t)rgb[i], serial);
                    }
                }
                break;
            // ジャイロセンサ
            case GYRO_SENSOR_RESET:
                {
                    uint8_t gyro_sensor_port = read_byte(serial);
                    ev3_gyro_sensor_reset(gyro_sensor_port);
                }
                break;
            case GYRO_SENSOR_GET_ANGLE:
                {
                    uint8_t gyro_sensor_port = read_byte(serial);
                    int16_t angle = ev3_gyro_sensor_get_angle(gyro_sensor_port);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)(angle >> 8), serial);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)angle, serial);
                }
                break;
            case GYRO_SENSOR_GET_RATE:
                {
                    uint8_t gyro_sensor_port = read_byte(serial);
                    int16_t angle = ev3_gyro_sensor_get_rate(gyro_sensor_port);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)(angle >> 8), serial);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)angle, serial);
                }
                break;
            // 超音波センサ
            case ULTRASONIC_SENSOR_GET_DISTANCE:
                {
                    uint8_t ultrasonic_sensor_port = read_byte(serial);
                    uint16_t distance = ev3_ultrasonic_sensor_get_distance(ultrasonic_sensor_port);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)(distance >> 8), serial);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)distance, serial);
                }
                break;
            case ULTRASONIC_SENSOR_LISTEN:
                {
                    uint8_t ultrasonic_sensor_port = read_byte(serial);
                    uint8_t listen = ev3_ultrasonic_sensor_listen(ultrasonic_sensor_port);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)listen, serial);
                }
                break;
            // 赤外線センサ
            case INFRARED_SENSOR_GET_DISTANCE:
                {
                    uint8_t infrared_sensor_port = read_byte(serial);
                    uint8_t distance = ev3_infrared_sensor_get_distance(infrared_sensor_port);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)distance, serial);
                }
                break;
            case INFRARED_SENSOR_GET_REMOTE:
                {
                    uint8_t infrared_sensor_port = read_byte(serial);
                    ir_remote_t ir_remote = ev3_infrared_sensor_get_remote(infrared_sensor_port);
                    int i;
                    for(i=0; i<4; i++) {
                        fputc((uint8_t)254, serial);
                        fputc((uint8_t)ir_remote.channel[i], serial);
                    }
                }
                break;
            case INFRARED_SENSOR_SEEK:
                {
                    uint8_t infrared_sensor_port = read_byte(serial);
                    ir_seek_t ir_seek = ev3_infrared_sensor_seek(infrared_sensor_port);
                    int i;
                    for(i=0; i<4; i++) {
                        fputc((uint8_t)254, serial);
                        fputc((uint8_t)ir_seek.heading[i], serial);
                        fputc((uint8_t)254, serial);
                        fputc((uint8_t)ir_seek.distance[i], serial);
                    }
                }
                break;
            // EV3本体
            case LCD_DRAW_STRING:
                {
                    char str[100];
                    uint8_t line = read_byte(serial);
                    int i = 0;
                    for (i=0; i<100; i++) {
                        uint8_t r = read_byte(serial);
                        str[i] = r;
                        if (r == 0) break;
                    }
                    strcpy(lcdstr, str);
                    ev3_lcd_draw_string(lcdstr, 0, fonth * line);
                }
                break;
            case BUTTON_IS_PRESSED:
                {
                    uint8_t button = read_byte(serial);
                    bool_t button_state = ev3_button_is_pressed(button);
                    if (button_state) {
                        button_state = 1;
                    }
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)button_state, serial);
                }
                break;
            default:
                break;
        }
    }
}
