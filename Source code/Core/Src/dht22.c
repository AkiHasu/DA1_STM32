#include "dht22.h"
#include <string.h> // Dùng cho memset

// --- Private Function Prototypes ---
static void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
static void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
static uint8_t DHT22_Start(void);
static uint8_t DHT22_Read_Response(void);

// --- Timer Initialization ---
void DHT_Delay_Init(void) {
    HAL_TIM_Base_Start(DHT_TIMER); // Khởi động timer đã cấu hình trong CubeMX
}

// --- Microsecond Delay ---
void DHT_Delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(DHT_TIMER, 0); // Reset bộ đếm timer
    while (__HAL_TIM_GET_COUNTER(DHT_TIMER) < us); // Chờ đủ số us
}

// --- Pin Mode Functions ---
static void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Output Push Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

static void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Kéo lên nội bộ (hoặc NOPULL nếu có trở ngoài)
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// --- DHT22 Start Signal ---
static uint8_t DHT22_Start(void) {
    Set_Pin_Output(DHT22_PORT, DHT22_PIN); // Set pin là output

    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_RESET); // Kéo xuống LOW
    DHT_Delay_us(1100); // Delay > 1ms (datasheet yêu cầu ít nhất 1ms)

    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET); // Kéo lên HIGH
    DHT_Delay_us(30); // Delay 20-40us (datasheet yêu cầu)

    Set_Pin_Input(DHT22_PORT, DHT22_PIN); // Set pin là input để đọc response
    return 1; // Giả sử thành công, việc kiểm tra response sẽ ở hàm khác
}

// --- DHT22 Check Response ---
static uint8_t DHT22_Read_Response(void) {
    uint8_t Response = 0;
    DHT_Delay_us(40); // Chờ khoảng 40us để DHT kéo xuống LOW

    // Kiểm tra xem pin có được kéo xuống LOW không (khoảng 80us)
    if (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) {
        DHT_Delay_us(80); // Chờ hết pha LOW
        // Kiểm tra xem pin có được kéo lên HIGH không (khoảng 80us)
        if ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) {
            Response = 1; // Có phản hồi
        }
        // Chờ hết pha HIGH (chờ đến khi pin xuống LOW sẵn sàng gửi data)
        while ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)));
    }
     // Nếu không có phản hồi đúng -> timeout (có thể thêm bộ đếm timeout ở đây)
    return Response;
}

// --- Read Data Function ---
uint8_t DHT22_Read_Data(float *Temperature, float *Humidity) {
    uint8_t data[5] = {0, 0, 0, 0, 0}; // 5 byte data (RH high, RH low, Temp high, Temp low, Checksum)
    uint8_t i, j;
    uint32_t timeout_counter;

    DHT22_Start(); // Gửi tín hiệu start

    if (!DHT22_Read_Response()) { // Kiểm tra phản hồi từ DHT22
         *Temperature = -999.0f; // Giá trị lỗi
         *Humidity = -999.0f;    // Giá trị lỗi
        return 0; // Không có phản hồi
    }

    // Đọc 40 bit (5 byte) data
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            // Chờ pin xuống LOW (bắt đầu bit, ~50us)
            timeout_counter = 0;
            while (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) {
                 DHT_Delay_us(1); // Delay nhỏ để tránh treo
                 timeout_counter++;
                 if (timeout_counter > 100) return 0; // Timeout chờ LOW
            }


            // Chờ pin lên HIGH và đo thời gian HIGH
            timeout_counter = 0;
             while ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) {
                DHT_Delay_us(1); // Delay nhỏ để đếm thời gian
                timeout_counter++;
                if (timeout_counter > 100) return 0; // Timeout chờ HIGH
            }

            // Nếu thời gian HIGH > 40us (thực tế thường ~70us) -> bit 1
            // Nếu thời gian HIGH < 40us (thực tế thường ~28us) -> bit 0
            if (timeout_counter > 40) {
                data[i] |= (1 << (7 - j)); // Ghi bit 1 vào byte data tương ứng
            }
             // Nếu là bit 0 thì không cần làm gì vì data[i] đã khởi tạo là 0
        }
    }

    // --- Checksum ---
    uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum == data[4]) {
        // --- Tính toán Nhiệt độ và Độ ẩm ---
        // Độ ẩm: (Byte RH_High << 8 | Byte RH_Low) / 10.0
        *Humidity = (float)(((uint16_t)data[0] << 8) | data[1]) / 10.0f;

        // Nhiệt độ:
        // Lấy 15 bit giá trị nhiệt độ (bỏ bit dấu)
        uint16_t temp_val = ((uint16_t)(data[2] & 0x7F) << 8) | data[3];
        *Temperature = (float)temp_val / 10.0f;

        // Kiểm tra bit dấu (bit 7 của byte Temp_High)
        if (data[2] & 0x80) {
            *Temperature = -(*Temperature); // Nhiệt độ âm
        }
        return 1; // Đọc thành công
    } else {
        // Checksum sai
         *Temperature = -998.0f; // Giá trị lỗi checksum
         *Humidity = -998.0f;    // Giá trị lỗi checksum
        return 0; // Checksum Error
    }
}
