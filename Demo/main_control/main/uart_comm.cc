#include "uart_comm.h"
#include "driver/uart.h"
#include "config.h"
#include <string.h>
#include "esp_log.h"

static const char* TAG = "UART";

UartComm& UartComm::getInstance() { static UartComm inst; return inst; }

void UartComm::init() {
    ESP_LOGI(TAG, "初始化串口通信...");
    uart_config_t cfg{};
    cfg.baud_rate = 115200;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, -1, -1);
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    ESP_LOGI(TAG, "串口通信初始化完成，波特率: %d", 115200);
}

void UartComm::send(const char* s) { 
    ESP_LOGV(TAG, "发送数据: %s", s);
    uart_write_bytes(UART_PORT, s, strlen(s)); 
}

int UartComm::read(char* buf, int len) { 
    int ret = uart_read_bytes(UART_PORT, (uint8_t*)buf, len, pdMS_TO_TICKS(10));
    if (ret > 0) {
        ESP_LOGV(TAG, "接收数据: %d 字节", ret);
    }
    return ret; 
}