#ifndef INC_DHT22_H_
#define INC_DHT22_H_

#include "main.h" // Include file main của project để có các định nghĩa HAL

// Định nghĩa Cổng và Chân GPIO bạn sử dụng cho DHT22
#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_0

// Định nghĩa Timer bạn dùng cho delay us (phải khớp với cấu hình IOC)
extern TIM_HandleTypeDef htim6; // Thay TIM6 bằng timer bạn đã chọn (ví dụ htim7)
#define DHT_TIMER &htim6

// --- Function Prototypes ---

// Hàm khởi tạo delay micro giây (nếu cần thiết, thường chỉ cần start timer)
void DHT_Delay_Init(void);

// Hàm delay micro giây
void DHT_Delay_us(uint16_t us);

// Hàm đọc dữ liệu từ DHT22
// Trả về 1 nếu thành công, 0 nếu thất bại (timeout, checksum)
uint8_t DHT22_Read_Data(float *Temperature, float *Humidity);

#endif /* INC_DHT22_H_ */
