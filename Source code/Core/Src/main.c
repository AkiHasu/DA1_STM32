#include "main.h"
#include <math.h>
#include "dht22.h"
#include <stdio.h>
#include <string.h>
#include "ILI9225.h"
#include "BH1750.h"
#include "MFRC522.h"
#include "ina219.h"

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

BH1750_device_t* bh1750_dev;
float Temperature = 0.0f;
float Humidity = 0.0f;
float lux_value = 0.0f;
uint8_t motion_detected = 0;
uint8_t bh1750_ok = 0;
float prev_lux_value = -100.0f;
uint8_t prev_motion_detected = 2;

// --- INA219 ---
INA219_t ina219_dev;
uint8_t ina219_ok = 0;
float bus_voltage_V = 0.0f;
float current_mA = 0.0f;
float power_mw = 0.0f;
float prev_bus_voltage_V = -999.0f;
float prev_current_mA = -999.0f;
float prev_power_mw = -999.0f;

// --- MFRC522 ---
#define MFRC522_UID_SIZE 5
uint8_t mfrc_status;
uint8_t mfrc_uid[MFRC522_UID_SIZE];
uint8_t mfrc_tag_present = 0;

// --- Authorized UIDs ---
const uint8_t authorized_uid_primary[4] = {0x63, 0xDF, 0x7B, 0x14};
#define MAX_SECONDARY_CARDS 2
uint8_t authorized_uids_secondary[MAX_SECONDARY_CARDS][4];
uint8_t num_secondary_cards_registered = 0;

typedef enum {
    MFRC522_STATE_IDLE,
    MFRC522_STATE_OPEN_DELAY,
	MFRC522_STATE_AWAITING_PRIMARY_FOR_REGISTRATION,
	MFRC522_STATE_REGISTRATION_MODE,
	MFRC522_STATE_SECONDARY_REGISTERED_SUCCESS_DELAY,
    MFRC522_STATE_POST_OPERATION_DELAY,
	MFRC522_STATE_AWAITING_DELETE_CONFIRM1,
	MFRC522_STATE_AWAITING_DELETE_CONFIRM2
} MFRC522_State_t;
static MFRC522_State_t mfrc522_state = MFRC522_STATE_IDLE;
static uint32_t mfrc522_delay_start_time = 0;
static int card_to_delete_index = -1;

// Thời gian trễ cho các trạng thái MFRC522
const uint32_t MFRC522_OPEN_DELAY_MS = 2000;
const uint32_t MFRC522_AWAIT_PRIMARY_DELAY_MS = 2000;
const uint32_t MFRC522_REGISTRATION_SUCCESS_DELAY_MS = 1000;
const uint32_t MFRC522_POST_OP_DELAY_MS = 300;
const uint32_t MFRC522_DELETE_WINDOW1_MS = 1000;
const uint32_t MFRC522_DELETE_WINDOW2_MS = 2000;

// --- LED ---
typedef enum {
    LED_STATE_OFF,
    LED_STATE_ON,
    LED_STATE_PENDING_OFF
} LedState_t;
static LedState_t led_state = LED_STATE_OFF;
static uint32_t led_off_pending_start_time = 0;
const uint32_t LED_OFF_DELAY_MS = 10000; // 10 giay

// --- LCD & Timing ---
char lcd_buf[30];
char lcd_line_buffer[40];
static uint32_t last_sensor_update_time = 0;
const uint32_t SENSOR_UPDATE_INTERVAL_MS = 500;
volatile uint8_t rcwl_update_flag = 0;
uint16_t calculate_motor_speed_from_temp(float temp);

// For initial LCD delay
static uint32_t lcd_init_delay_start_time = 0;
static uint8_t lcd_init_delay_active = 1; // Flag to indicate if initial delay is active
const uint32_t LCD_INIT_STABILIZE_DELAY_MS = 100;


// --- Defines ---
#define LED_PORT GPIOC
#define LED_PIN GPIO_PIN_5
#define MOTION_PORT GPIOA
#define MOTION_PIN GPIO_PIN_1
#define VOLTAGE_CHANGE_THRESHOLD 0.05f
#define CURRENT_CHANGE_THRESHOLD 1.0f
#define POWER_CHANGE_THRESHOLD 2.0f

#define MOTOR_IN1_PORT GPIOA
#define MOTOR_IN1_PIN GPIO_PIN_6
#define MOTOR_IN2_PORT GPIOB
#define MOTOR_IN2_PIN GPIO_PIN_9
#define MOTOR_ENA_TIMER htim2
#define MOTOR_ENA_CHANNEL TIM_CHANNEL_1
#define MOTOR_PWM_MAX_DUTY 59999

// --- Servo Defines (Door Control) ---
#define SERVO_TIMER htim1
#define SERVO_CHANNEL TIM_CHANNEL_3
#define SERVO_PULSE_CLOSED 1900 // Pulse for closed position
#define SERVO_PULSE_OPEN   3900 // Pulse for open position

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);

void Error_Handler(void);
void motor_stop();
void motor_set_speed(uint16_t speed);

// Function Prototypes for refactored handlers
void Handle_LCD_Init(uint32_t current_time);
void Handle_RCWL0516(void);
void Handle_RFID(uint32_t current_time);
void Handle_DHT22(void);
void Handle_BH1750(void);
void Handle_INA219(void);
void Handle_Motor(void);
void Handle_LED(uint32_t current_time);
void Handle_Servo(uint8_t open_command); // 1 to open, 0 to close


void Handle_LCD_Init(uint32_t current_time) {
    if (lcd_init_delay_active) {
        if (current_time - lcd_init_delay_start_time >= LCD_INIT_STABILIZE_DELAY_MS) {
            if (!bh1750_ok) {
                draw_string(5, 140, COLOR_ORANGE, 1, "BH1750 Fail!");
            }
            if (!ina219_ok) {
                draw_string(5, 155, COLOR_ORANGE, 1, "INA219 Fail!");
            }
            lcd_init_delay_active = 0; // Delay finished
        }
    }
}

void Handle_RCWL0516(void) {
    if (rcwl_update_flag) {
        rcwl_update_flag = 0;
        if (motion_detected != prev_motion_detected) {
            if (motion_detected == GPIO_PIN_SET) {
                sprintf(lcd_line_buffer, "Motion: YES ");
            } else {
                sprintf(lcd_line_buffer, "Motion: NO  "); // Ensure same length for overwrite
            }
            uint16_t motion_color = (motion_detected == GPIO_PIN_SET) ? COLOR_GREEN : COLOR_RED;
            draw_fast_string(5, 40, motion_color, COLOR_BLACK, lcd_line_buffer);
            prev_motion_detected = motion_detected;
        }
    }
}

void Handle_Servo(uint8_t open_command) {
    if (open_command) {
        __HAL_TIM_SET_COMPARE(&SERVO_TIMER, SERVO_CHANNEL, SERVO_PULSE_OPEN); // Mở cửa
    } else {
        __HAL_TIM_SET_COMPARE(&SERVO_TIMER, SERVO_CHANNEL, SERVO_PULSE_CLOSED); // Đóng cửa
    }
}

void Handle_RFID(uint32_t current_time) {
    switch (mfrc522_state) {
        case MFRC522_STATE_IDLE:
            mfrc_tag_present = 0;
            memset(mfrc_uid, 0, MFRC522_UID_SIZE);
            mfrc_status = MFRC522_Request(PICC_REQIDL, mfrc_uid);
            if (mfrc_status == MI_OK) {
                mfrc_status = MFRC522_Anticoll(mfrc_uid);
                if (mfrc_status == MI_OK) {
                    mfrc_tag_present = 1;
                }
            }

            if (mfrc_tag_present) {
                uint8_t access_granted = 0;
                int secondary_card_scanned_index = -1;
                if (memcmp(mfrc_uid, authorized_uid_primary, 4) == 0) {
                    access_granted = 1;
                } else {
                    for (int i = 0; i < num_secondary_cards_registered; i++) {
                        if (memcmp(mfrc_uid, authorized_uids_secondary[i], 4) == 0) {
                            secondary_card_scanned_index = i;
                            break;
                        }
                    }
                }

                if (access_granted) {
                    draw_fast_string(30, 100, COLOR_GREEN, COLOR_BLACK, "OPEN SUCCESSFULLY ");
                    Handle_Servo(1); // Mở cửa
                    mfrc522_delay_start_time = current_time;
                    mfrc522_state = MFRC522_STATE_OPEN_DELAY;
                } else if (secondary_card_scanned_index != -1) {
                    draw_fast_string(10, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                    draw_fast_string(50, 100, COLOR_YELLOW, COLOR_BLACK, "SUB CARD");
                    card_to_delete_index = secondary_card_scanned_index;
                    mfrc522_delay_start_time = current_time;
                    mfrc522_state = MFRC522_STATE_AWAITING_DELETE_CONFIRM1;
                } else {
                    draw_fast_string(10, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                    draw_fast_string(50, 100, COLOR_RED, COLOR_BLACK, "  WRONG CARD   ");
                    mfrc522_delay_start_time = current_time;
                    mfrc522_state = MFRC522_STATE_AWAITING_PRIMARY_FOR_REGISTRATION;
                }
            }
            break;

        case MFRC522_STATE_OPEN_DELAY:
            if (current_time - mfrc522_delay_start_time >= MFRC522_OPEN_DELAY_MS) {
                Handle_Servo(0); // Đóng cửa
                draw_fast_string(30, 100, COLOR_BLACK, COLOR_BLACK, "                                                ");
                mfrc522_delay_start_time = current_time;
                mfrc522_state = MFRC522_STATE_POST_OPERATION_DELAY;
            }
            break;

        case MFRC522_STATE_AWAITING_PRIMARY_FOR_REGISTRATION:
            if (current_time - mfrc522_delay_start_time >= MFRC522_AWAIT_PRIMARY_DELAY_MS) {
                draw_fast_string(50, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                mfrc522_delay_start_time = current_time;
                mfrc522_state = MFRC522_STATE_POST_OPERATION_DELAY;
            } else {
                mfrc_tag_present = 0;
                mfrc_status = MFRC522_Request(PICC_REQIDL, mfrc_uid);
                if (mfrc_status == MI_OK) {
                    mfrc_status = MFRC522_Anticoll(mfrc_uid);
                    if (mfrc_status == MI_OK) {
                        mfrc_tag_present = 1;
                    }
                }
                if (mfrc_tag_present) {
                    if (memcmp(mfrc_uid, authorized_uid_primary, 4) == 0) {
                        draw_fast_string(10, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                        draw_fast_string(50, 100, COLOR_BLUE, COLOR_BLACK, "REGISTER CARD");
                        mfrc522_state = MFRC522_STATE_REGISTRATION_MODE;
                    }
                    HAL_Delay(300);
                }
            }
            break;

        case MFRC522_STATE_REGISTRATION_MODE:
            mfrc_tag_present = 0;
            mfrc_status = MFRC522_Request(PICC_REQIDL, mfrc_uid);
            if (mfrc_status == MI_OK) {
                mfrc_status = MFRC522_Anticoll(mfrc_uid);
                if (mfrc_status == MI_OK) {
                    mfrc_tag_present = 1;
                }
            }
            if (mfrc_tag_present) {
                if (memcmp(mfrc_uid, authorized_uid_primary, 4) == 0) {
                    draw_fast_string(50, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                    mfrc522_delay_start_time = current_time;
                    mfrc522_state = MFRC522_STATE_POST_OPERATION_DELAY;
                } else {
                    uint8_t already_registered_as_secondary = 0;
                    for (int i = 0; i < num_secondary_cards_registered; i++) {
                        if (memcmp(mfrc_uid, authorized_uids_secondary[i], 4) == 0) {
                            already_registered_as_secondary = 1;
                            break;
                        }
                    }
                    if (already_registered_as_secondary) {
                        draw_fast_string(50, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                        draw_fast_string(50, 100, COLOR_ORANGE, COLOR_BLACK, "CARD EXISTENCE");
                        mfrc522_delay_start_time = current_time;
                        mfrc522_state = MFRC522_STATE_SECONDARY_REGISTERED_SUCCESS_DELAY;
                    } else {
                        if (num_secondary_cards_registered < MAX_SECONDARY_CARDS) {
                            memcpy(authorized_uids_secondary[num_secondary_cards_registered], mfrc_uid, 4);
                            num_secondary_cards_registered++;
                            draw_fast_string(50, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                            draw_fast_string(50, 100, COLOR_GREEN, COLOR_BLACK, "ADD SUCCESSFULL");
                            mfrc522_delay_start_time = current_time;
                            mfrc522_state = MFRC522_STATE_SECONDARY_REGISTERED_SUCCESS_DELAY;
                        } else {
                            draw_fast_string(50, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                            draw_fast_string(50, 100, COLOR_ORANGE, COLOR_BLACK, "ENOUGH CARDS");
                            mfrc522_delay_start_time = current_time;
                            mfrc522_state = MFRC522_STATE_SECONDARY_REGISTERED_SUCCESS_DELAY;
                        }
                    }
                }
            }
            break;

        case MFRC522_STATE_SECONDARY_REGISTERED_SUCCESS_DELAY:
            if (current_time - mfrc522_delay_start_time >= MFRC522_REGISTRATION_SUCCESS_DELAY_MS) {
                draw_fast_string(50, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                mfrc522_delay_start_time = current_time;
                mfrc522_state = MFRC522_STATE_POST_OPERATION_DELAY;
            }
            break;

        case MFRC522_STATE_POST_OPERATION_DELAY:
            if (current_time - mfrc522_delay_start_time >= MFRC522_POST_OP_DELAY_MS) {
                mfrc522_state = MFRC522_STATE_IDLE;
            }
            break;

        case MFRC522_STATE_AWAITING_DELETE_CONFIRM1:
            if (current_time - mfrc522_delay_start_time >= MFRC522_DELETE_WINDOW1_MS) {
                draw_fast_string(30, 100, COLOR_GREEN, COLOR_BLACK, "OPEN SUCCESSFULLY ");
                Handle_Servo(1); // Mở cửa
                mfrc522_delay_start_time = current_time;
                mfrc522_state = MFRC522_STATE_OPEN_DELAY;
                card_to_delete_index = -1;
            } else {
                mfrc_tag_present = 0;
                mfrc_status = MFRC522_Request(PICC_REQIDL, mfrc_uid);
                if (mfrc_status == MI_OK) {
                    mfrc_status = MFRC522_Anticoll(mfrc_uid);
                    if (mfrc_status == MI_OK) {
                        mfrc_tag_present = 1;
                    }
                }
                if (mfrc_tag_present && memcmp(mfrc_uid, authorized_uid_primary, 4) == 0) {
                    draw_fast_string(50, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                    draw_fast_string(50, 100, COLOR_YELLOW, COLOR_BLACK, "DELETE SUB CARD?");
                    mfrc522_delay_start_time = current_time;
                    HAL_Delay(300);
                    mfrc522_state = MFRC522_STATE_AWAITING_DELETE_CONFIRM2;
                }
            }
            break;

        case MFRC522_STATE_AWAITING_DELETE_CONFIRM2:
            if (current_time - mfrc522_delay_start_time >= MFRC522_DELETE_WINDOW2_MS) {
                draw_fast_string(50, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                mfrc522_state = MFRC522_STATE_POST_OPERATION_DELAY;
                card_to_delete_index = -1;
            } else {
                mfrc_tag_present = 0;
                mfrc_status = MFRC522_Request(PICC_REQIDL, mfrc_uid);
                if (mfrc_status == MI_OK) {
                    mfrc_status = MFRC522_Anticoll(mfrc_uid);
                    if (mfrc_status == MI_OK) {
                        mfrc_tag_present = 1;
                    }
                }
                if (mfrc_tag_present && memcmp(mfrc_uid, authorized_uid_primary, 4) == 0) {
                    if (card_to_delete_index != -1 && card_to_delete_index < num_secondary_cards_registered) {
                        for (int i = card_to_delete_index; i < num_secondary_cards_registered - 1; i++) {
                            memcpy(authorized_uids_secondary[i], authorized_uids_secondary[i+1], 4);
                        }
                        if (num_secondary_cards_registered > 0) {
                            memset(authorized_uids_secondary[num_secondary_cards_registered - 1], 0x00, 4);
                        }
                        num_secondary_cards_registered--;
                        draw_fast_string(50, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                        draw_fast_string(50, 100, COLOR_GREEN, COLOR_BLACK, "DELETE SUCCESSFULL");
                    } else {
                        draw_fast_string(50, 100, COLOR_BLACK, COLOR_BLACK, "                         ");
                        draw_fast_string(50, 100, COLOR_RED, COLOR_BLACK, "DELETE FAIL");
                    }
                    card_to_delete_index = -1;
                    mfrc522_delay_start_time = current_time;
                    mfrc522_state = MFRC522_STATE_SECONDARY_REGISTERED_SUCCESS_DELAY;
                    HAL_Delay(300);
                }
            }
            break;
    }
}

void Handle_DHT22(void) {
    if (DHT22_Read_Data(&Temperature, &Humidity)) {
        sprintf(lcd_buf, "Temp:%.1fC ", Temperature);
        draw_fast_string(5, 5, COLOR_RED, COLOR_BLACK, lcd_buf);
        sprintf(lcd_buf, "Humi:%.1f%% ", Humidity);
        draw_fast_string(115, 5, COLOR_BLUE, COLOR_BLACK, lcd_buf);
    } else {
        draw_fast_string(5, 5, COLOR_RED, COLOR_BLACK, "DHT22 Error");
        draw_fast_string(115, 5, COLOR_BLACK, COLOR_BLACK, "           "); // Clear Humi area
    }
}

void Handle_BH1750(void) {
    uint8_t prev_bh1750_ok_status = bh1750_ok;
    if (bh1750_ok) {
        lux_value = BH1750_get_lumen(bh1750_dev);
        if (lux_value < 0) { // Assuming -1.0f or similar indicates read error
           // bh1750_ok = 0; // Potentially mark as not OK if read fails, or rely on init check
        }
    }
     if (!bh1750_ok) { // If init failed or a read makes it not okay
        lux_value = -1.0f; // Consistent error value
    }
    bool bh1750_status_just_changed = (bh1750_ok != prev_bh1750_ok_status);

    if (fabs(lux_value - prev_lux_value) > 1.0f || bh1750_status_just_changed) {
        if (bh1750_ok && lux_value >= 0) {
            sprintf(lcd_line_buffer, "Lux:%.1f  ", lux_value);
        } else {
            sprintf(lcd_line_buffer, "Lux:Error ");
        }
        draw_fast_string(5, 20, COLOR_YELLOW, COLOR_BLACK, lcd_line_buffer);
        prev_lux_value = lux_value;
    }
}

void Handle_INA219(void) {
    uint8_t prev_ina219_ok_status_for_cycle = ina219_ok;
    uint16_t bus_mv = 0;
    int16_t current_raw_ma = 0;
    uint16_t power_raw_mw = 0;

    if (ina219_ok) {
        bus_mv = INA219_ReadBusVoltage(&ina219_dev);
        if (bus_mv == 0xFFFF) { // Check for I2C read error indicator from INA219 library
            ina219_ok = 0; // Mark as not OK if read fails
        }

        if (ina219_ok) { // Proceed only if still OK
            current_raw_ma = INA219_ReadCurrent(&ina219_dev);
            // Add error check for INA219_ReadCurrent if applicable
            power_raw_mw = INA219_ReadPower(&ina219_dev);
            // Add error check for INA219_ReadPower if applicable
        }
    }

    if (ina219_ok) {
        bus_voltage_V = (float)bus_mv / 1000.0f;
        current_mA = (float)current_raw_ma;
        power_mw = (float)power_raw_mw;
    } else { // If init failed or a read makes it not okay
        bus_voltage_V = -1.0f;
        current_mA = -1.0f;
        power_mw = -1.0f;
    }
    bool ina219_status_just_changed = (ina219_ok != prev_ina219_ok_status_for_cycle);

    if (fabs(bus_voltage_V - prev_bus_voltage_V) > VOLTAGE_CHANGE_THRESHOLD || ina219_status_just_changed) {
        if (ina219_ok && bus_voltage_V >= 0) {
            sprintf(lcd_line_buffer, "V:%.2fV  ", bus_voltage_V);
        } else {
            sprintf(lcd_line_buffer, "V: Error ");
        }
        draw_fast_string(5, 60, COLOR_CYAN, COLOR_BLACK, lcd_line_buffer);
        prev_bus_voltage_V = bus_voltage_V;
    }

    if (fabs(current_mA - prev_current_mA) > CURRENT_CHANGE_THRESHOLD || ina219_status_just_changed) {
        if (ina219_ok) {
            sprintf(lcd_line_buffer, "I:%.1fmA  ", current_mA);
        } else {
            sprintf(lcd_line_buffer, "I: Error  ");
        }
        draw_fast_string(115, 60, COLOR_MAGENTA, COLOR_BLACK, lcd_line_buffer);
        prev_current_mA = current_mA;
    }

    if (fabs(power_mw - prev_power_mw) > POWER_CHANGE_THRESHOLD || ina219_status_just_changed) {
        if (ina219_ok && power_mw >= 0) {
            sprintf(lcd_line_buffer, "P:%.1fmW  ", power_mw);
        } else {
            sprintf(lcd_line_buffer, "P: Error  ");
        }
        draw_fast_string(5, 80, COLOR_GREEN, COLOR_BLACK, lcd_line_buffer);
        prev_power_mw = power_mw;
    }
}

void Handle_Motor(void) {
    if (motion_detected == GPIO_PIN_SET) {
        uint16_t speed = calculate_motor_speed_from_temp(Temperature);
        motor_set_speed(speed);
    } else {
        motor_stop();
    }
}

void Handle_LED(uint32_t current_time) {
    bool should_led_be_on = (bh1750_ok && lux_value >= 0 && lux_value < 300.0f && motion_detected == GPIO_PIN_SET);
    if (should_led_be_on) {
        if (led_state != LED_STATE_ON) {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            led_state = LED_STATE_ON;
        }
    } else { // should_led_be_on is false
        switch (led_state) {
            case LED_STATE_ON:
                led_state = LED_STATE_PENDING_OFF;
                led_off_pending_start_time = current_time;
                break;
            case LED_STATE_PENDING_OFF:
                if (current_time - led_off_pending_start_time >= LED_OFF_DELAY_MS - 6850) { // Giữ lại logic trễ hiện tại
                    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
                    led_state = LED_STATE_OFF;
                }
                break;
            case LED_STATE_OFF:
                // Do nothing
                break;
        }
    }
}


int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM6_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();
    MX_SPI2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM7_Init();

    for (int i = 0; i < MAX_SECONDARY_CARDS; i++) {
            for (int j = 0; j < 4; j++) {
                authorized_uids_secondary[i][j] = 0x00;
        }
    }
    num_secondary_cards_registered = 0;

    DHT_Delay_Init();
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // For Servo
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // For Motor
    HAL_TIM_Base_Start_IT(&htim7);        // For RCWL0516 poll

    MFRC522_Init();
    lcd_init();

    bh1750_dev = BH1750_init_dev_struct(&hi2c1, "BH1750_Sensor", true);
    if (bh1750_dev == NULL) {
        bh1750_ok = 0;
    } else {
        if (BH1750_init_dev(bh1750_dev) != HAL_OK) {
            bh1750_ok = 0;
        } else {
            bh1750_ok = 1;
        }
    }

    if (INA219_Init(&ina219_dev, &hi2c1, (INA219_ADDRESS << 1)) == 1) {
        ina219_ok = 1;
    } else {
        ina219_ok = 0;
    }

    fill_rectangle(0, 0, WIDTH - 1, HEIGHT - 1, COLOR_BLACK);
    lcd_init_delay_start_time = HAL_GetTick();
    lcd_init_delay_active = 1;

    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    led_state = LED_STATE_OFF;
    motor_stop();
    Handle_Servo(0); // Initialize Servo to closed position (Đóng cửa)

    last_sensor_update_time = HAL_GetTick();

    while (1)
    {
        uint32_t current_time = HAL_GetTick();

        Handle_LCD_Init(current_time);
        if (lcd_init_delay_active) {
            // Still waiting for LCD to stabilize, skip other operations in this iteration
            continue;
        }

        Handle_RCWL0516();          // Cập nhật hiển thị cảm biến chuyển động
        Handle_RFID(current_time);  // Xử lý logic RFID và điều khiển Servo cửa

        // --- Sensor Updates Interval ---
        if (current_time - last_sensor_update_time >= SENSOR_UPDATE_INTERVAL_MS)
        {
            last_sensor_update_time = current_time;

            Handle_DHT22();         // Đọc và hiển thị dữ liệu DHT22
            Handle_BH1750();        // Đọc và hiển thị dữ liệu BH1750
            Handle_INA219();        // Đọc và hiển thị dữ liệu INA219

            // Các hàm điều khiển phụ thuộc vào dữ liệu cảm biến mới nhất và trạng thái chuyển động
            Handle_Motor();         // Điều khiển động cơ dựa trên nhiệt độ và chuyển động
            Handle_LED(current_time); // Điều khiển LED dựa trên ánh sáng và chuyển động
        }
    } // End of while(1)
}

uint16_t calculate_motor_speed_from_temp(float temp) {
    uint16_t pwm_value = 0;
    float percentage = 0.0f;
    if (temp < 22.5f) {
        percentage = 0.0f;
    } else if (temp < 23.0f) {
        percentage = 0.11f;
    } else if (temp < 23.8f) {
        percentage = 0.22f;
    } else if (temp < 24.5f) {
        percentage = 0.33f;
    } else if (temp < 29.5f) {
        percentage = 0.44f;
    } else if (temp < 30.5f) {
        percentage = 0.56f;
    } else if (temp < 31.0f) {
        percentage = 0.67f;
    } else if (temp < 32.0f) {
        percentage = 0.78f;
    } else if (temp < 33.0f) {
        percentage = 0.89f;
    } else { // temp >= 33
        percentage = 1.00f;
    }
    pwm_value = (uint16_t)(percentage * MOTOR_PWM_MAX_DUTY);
    if (pwm_value > MOTOR_PWM_MAX_DUTY) {
        pwm_value = MOTOR_PWM_MAX_DUTY;
    }
    return pwm_value;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        motion_detected = HAL_GPIO_ReadPin(MOTION_PORT, MOTION_PIN);
        rcwl_update_flag = 1;
    }
}

void motor_stop() {
    HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&MOTOR_ENA_TIMER, MOTOR_ENA_CHANNEL, 0);
}

void motor_set_speed(uint16_t speed) {
    HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);

    if (speed > MOTOR_PWM_MAX_DUTY) {
        speed = MOTOR_PWM_MAX_DUTY;
    }
    __HAL_TIM_SET_COMPARE(&MOTOR_ENA_TIMER, MOTOR_ENA_CHANNEL, speed);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_SPI1_Init(void)
    {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_1LINE;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_SPI2_Init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_TIM1_Init(void) // Timer for Servo
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 83;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 19999; // 50Hz
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = SERVO_PULSE_CLOSED; // Initial pulse for servo (closed)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, SERVO_CHANNEL) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void) // Timer for Motor PWM
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = MOTOR_PWM_MAX_DUTY;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; // Initial duty cycle 0
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, MOTOR_ENA_CHANNEL) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 83;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 65535;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_TIM7_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim7.Instance = TIM7;
    // PCLK1 (cho TIM2-7) = HCLK / APB1CLKDivider = (336/2) / 4 = 168MHz / 4 = 42MHz.
    // Nếu SystemCoreClock là 168MHz, thì APB1 Timer Clock (TIM2,3,4,5,6,7,12,13,14) là PCLK1 * 2 = 84MHz (nếu APB1 Prescaler != 1)
    // Giả sử APB1 Timer Clock là 84MHz
    // Prescaler = 8400 - 1 => Clock timer = 84MHz / 8400 = 10kHz
    // Period = 500 - 1 => Tần số ngắt = 10kHz / 500 = 20Hz => Chu kỳ 50ms
    htim7.Init.Prescaler = 8400 - 1;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 500 -1;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}


static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, TFT_RST_Pin|TFT_CS_Pin|TFT_RS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MFRC522_SDA_GPIO_Port, MFRC522_SDA_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, DHT22_PIN_Pin | MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MFRC522_RST_Pin | MOTOR_IN2_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = DHT22_PIN_Pin|TFT_RST_Pin|TFT_CS_Pin|TFT_RS_Pin | MOTOR_IN1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTION_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MOTION_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MFRC522_RST_Pin|MFRC522_SDA_Pin|MOTOR_IN2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MFRC522_IRQ_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MFRC522_IRQ_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
