// 標準ライブラリ
#include <string.h>

// EV3RT C++ライブラリ
#include <Port.h>
#include <Clock.h>
#include <Sensor.h>
#include <SonarSensor.h>
#include <TouchSensor.h>
#include <ColorSensor.h>
#include <GyroSensor.h>
#include <Motor.h>
#include <Steering.h>

// EV3RT Cライブラリ
#include "ev3api.h"

// アプリケーションヘッダ
#include "app.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

using namespace ev3api;

FILE* serial = NULL;
uint64_t last_updated_time;
Clock clock;
Motor* motor[NUM_PORT_M];
Sensor* sensor[NUM_PORT_S];
Steering* steering;
int enable_watchdog_task = 0;

/**
 * Get current time in [msec].
 */
uint64_t get_time()
{
    static uint64_t start = -1;
    uint64_t time;
    time = clock.now();
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
    if (steering == nullptr) return;
    if (enable_watchdog_task != 1) return;
    ulong_t now = get_time();
    if (now - last_updated_time > 1000) {
        steering->setPower(0, 0);
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
                     ePortM motor_port = (ePortM)read_byte(serial);
                     motor_type_t motor_type = (motor_type_t)read_byte(serial);
                     if(motor[(int)motor_port] != nullptr)
                         delete motor[(int)motor_port];
                     motor[(int)motor_port] = new Motor(motor_port, false, motor_type);
                }
                break;
            case MOTOR_STEER:
                {
                    ePortM motor_port;
                    motor_port = (ePortM)read_byte(serial); // left
                    if(motor[(int)motor_port] == nullptr) return;
                    motor_port = (ePortM)read_byte(serial); // right
                    if(motor[(int)motor_port] == nullptr) return;

                    int drive = read_byte(serial) - 100;
                    int steer = read_byte(serial) - 100;
                    if (drive > 100) drive=100;
                    if (drive < -100) drive=-100;
                    if (steer > 100) steer=100;
                    if (steer < -100) steer=-100;
                    if(steering == nullptr) return;
                    steering->setPower(drive, steer);
                }
                break;
            case MOTOR_GET_COUNTS:
                {
                    motor_port_t motor_port = (motor_port_t)read_byte(serial);
                    if(motor[(int)motor_port] == nullptr) return;
                    int32_t counts = ev3_motor_get_counts(motor_port);
                    int i;
                    for(i=3; i>0; i--) {
                        fputc((uint8_t)254, serial);
                        fputc((uint8_t)(counts >> (8 * i)), serial);
                    }
                }
                break;
            case MOTOTR_GET_POWER:
                {
                    motor_port_t motor_port = (motor_port_t)read_byte(serial);
                    if(motor[(int)motor_port] == nullptr) return;
                    int counts = ev3_motor_get_power(motor_port);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)counts , serial);
                }
                break;
            case MOTOR_RESET_COUNTS:
                {
                    ePortM motor_port = (ePortM)read_byte(serial);
                    if(motor[(int)motor_port] == nullptr) return;
                    motor[(int)motor_port]->reset();
                }
                break;
            case MOTOR_ROTATE:
                {
                    ePortM motor_port = (ePortM)read_byte(serial);
                    if(motor[(int)motor_port] == nullptr) return;

                    uint8_t degrees = read_byte(serial);
                    uint32_t speed_abs = read_byte(serial);
                    uint8_t blocking = read_byte(serial);
                    // 握りつぶし
                    speed_abs *= blocking;
                    motor[(int)motor_port]->setCount((int32_t)degrees);
                }
                break;
            case MOTOR_SET_POWER:
                {
                    ePortM motor_port = (ePortM)read_byte(serial);
                    if(motor[(int)motor_port] == nullptr) return;
                    int power = read_byte(serial);
                    motor[(int)motor_port]->setPWM(power);
                }
                break;
            case MOTOR_STOP:
                {
                    ePortM motor_port = (ePortM)read_byte(serial);
                    if(motor[(int)motor_port] == nullptr) return;
                    uint8_t breaking = read_byte(serial);
                    // 握りつぶし
                    breaking *= 0;
                    motor[(int)motor_port]->stop();
                }
                break;
            // センサ共通
            case SENSOR_CONFIG:
                {
                    ePortS sensor_port = (ePortS)read_byte(serial);
                    sensor_type_t sensor_type = (sensor_type_t)read_byte(serial);
                    if(sensor[(int)sensor_port] != nullptr) return;
                    switch(sensor_type)
                    {
                        case ULTRASONIC_SENSOR:
                            sensor[(int)sensor_port] = new SonarSensor(sensor_port);
                            break;
                        case GYRO_SENSOR:
                            sensor[(int)sensor_port] = new GyroSensor(sensor_port);
                            break;
                        case TOUCH_SENSOR:
                            sensor[(int)sensor_port] = new TouchSensor(sensor_port);
                            break;
                        case COLOR_SENSOR:
                            sensor[(int)sensor_port] = new ColorSensor(sensor_port);
                            break;
                        default:
                            break;
                    }
                }
                break;
            // タッチセンサ
            case TOUCH_SENSOR_IS_PRESSED:
                {
                    ePortS touch_sensor_port = (ePortS)read_byte(serial);
                    if(sensor[(int)touch_sensor_port] == nullptr)
                        return;
                    bool touch = ((TouchSensor *)sensor[(int)touch_sensor_port])->isPressed();
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)touch, serial);
                }
                break;
            // カラーセンサ
            case COLOR_SENSOR_GET_REFLECT:
                {
                    ePortS color_sensor_port = (ePortS)read_byte(serial);
                    if(sensor[(int)color_sensor_port] == nullptr)
                        return;
                    int8_t color = ((ColorSensor *)sensor[(int)color_sensor_port])->getBrightness();
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)color, serial);
                }
                break;
            case COLOR_SENSOR_GET_AMBIENT:
                {
                    ePortS color_sensor_port = (ePortS)read_byte(serial);
                    if(sensor[(int)color_sensor_port] == nullptr)
                        return;
                    uint8_t ambient = ((ColorSensor *)sensor[(int)color_sensor_port])->getAmbient();
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)ambient, serial);
                }
                break;
            case COLOR_SENSOR_GET_COLOR:
                {
                    ePortS color_sensor_port = (ePortS)read_byte(serial);
                    if(sensor[(int)color_sensor_port] == nullptr)
                        return;
                    colorid_t  color = ((ColorSensor *)sensor[(int)color_sensor_port])->getColorNumber();
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)color, serial);
                }
                break;
            case COLOR_SENSOR_GET_RGB_RAW:
                {
                    ePortS color_sensor_port = (ePortS)read_byte(serial);
                    if(sensor[(int)color_sensor_port] == nullptr)
                        return;
                    rgb_raw_t rgb_raw;
                    uint16_t rgb[3];
                    ((ColorSensor *)sensor[(int)color_sensor_port])->getRawColor(rgb_raw);
                    rgb[0] = rgb_raw.r;
                    rgb[1] = rgb_raw.g;
                    rgb[2] = rgb_raw.b;
                    for(int i=0; i<3; i++){
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
                    ePortS gyro_sensor_port = (ePortS)read_byte(serial);
                    if(sensor[(int)gyro_sensor_port] == nullptr)
                        return;
                    ((GyroSensor *)sensor[(int)gyro_sensor_port])->reset();
                }
                break;
            case GYRO_SENSOR_GET_ANGLE:
                {
                    ePortS gyro_sensor_port = (ePortS)read_byte(serial);
                    if(sensor[(int)gyro_sensor_port] == nullptr)
                        return;
                    int16_t angle = ((GyroSensor *)sensor[(int)gyro_sensor_port])->getAngle();
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)(angle >> 8), serial);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)angle, serial);
                }
                break;
            case GYRO_SENSOR_GET_RATE:
                {
                    ePortS gyro_sensor_port = (ePortS)read_byte(serial);
                    if(sensor[(int)gyro_sensor_port] == nullptr)
                        return;
                    int16_t rate = ((GyroSensor *)sensor[(int)gyro_sensor_port])->getAnglerVelocity();
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)(rate >> 8), serial);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)rate, serial);
                }
                break;
            // 超音波センサ
            case ULTRASONIC_SENSOR_GET_DISTANCE:
                {
                    ePortS ultrasonic_sensor_port = (ePortS)read_byte(serial);
                    if(sensor[(int)ultrasonic_sensor_port] == nullptr)
                        return;
                    int16_t distance = ((SonarSensor *)sensor[(int)ultrasonic_sensor_port])->getDistance();
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)(distance >> 8), serial);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)distance, serial);
                }
                break;
            case ULTRASONIC_SENSOR_LISTEN:
                {
                    ePortS ultrasonic_sensor_port = (ePortS)read_byte(serial);
                    if(sensor[(int)ultrasonic_sensor_port] == nullptr)
                        return;
                    bool listen = ((SonarSensor *)sensor[(int)ultrasonic_sensor_port])->listen();
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)listen, serial);
                }
                break;
            // 赤外線センサ
            case INFRARED_SENSOR_GET_DISTANCE:
                {
                    sensor_port_t infrared_sensor_port = (sensor_port_t)read_byte(serial);
                    uint8_t distance = ev3_infrared_sensor_get_distance(infrared_sensor_port);
                    fputc((uint8_t)254, serial);
                    fputc((uint8_t)distance, serial);
                }
                break;
            case INFRARED_SENSOR_GET_REMOTE:
                {
                    sensor_port_t infrared_sensor_port = (sensor_port_t)read_byte(serial);
                    ir_remote_t ir_remote = ev3_infrared_sensor_get_remote(infrared_sensor_port);
                    for(int i=0; i<4; i++) {
                        fputc((uint8_t)254, serial);
                        fputc((uint8_t)ir_remote.channel[i], serial);
                    }
                }
                break;
            case INFRARED_SENSOR_SEEK:
                {
                    sensor_port_t infrared_sensor_port = (sensor_port_t)read_byte(serial);
                    ir_seek_t ir_seek = ev3_infrared_sensor_seek(infrared_sensor_port);
                    for(int i=0; i<4; i++) {
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
                    for (int i=0; i<100; i++) {
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
                    button_t button = (button_t)read_byte(serial);
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
