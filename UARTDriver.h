#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <Arduino.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "Event.h"

// #define UART_NUM UART_NUM_1 // 使用UART1
#define TX_PIN 43 // UART发送引脚
#define RX_PIN 44 // UART接收引脚
// #define BUFFER_SIZE 1024 // 缓冲区大小

class UARTDriver
{
private:
    bool is_begin = false;

    uart_port_t uart_num;            // UART端口
    int tx_pin;                      // 发送引脚
    int rx_pin;                      // 接收引脚
    int buffer_size;                 // 缓冲区大小
    QueueHandle_t uart_queue;        // UART队列
    TaskHandle_t task_handle = NULL; // 任务句柄

    uart_config_t uart_config; // UART配置

    // UART 默认事件处里
    Event<uart_event_t> onDefault;

    // UART接收事件
    Event<uart_event_t> onReceive;

    // UART RX缓冲区满
    Event<uart_event_t> onBufferFull;

    // UART FIFO溢出
    Event<uart_event_t> onFifoOverflow;

    void uart_event_task(void *pvParameters) // UART事件处理任务
    {
        uart_event_t event;
        // uint8_t data[buffer_size];
        while (true)
        {
            // 等待UART事件
            if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
            {
                switch (event.type)
                {
                case UART_DATA:
                    // 读取数据
                    // int len = uart_read_bytes(uart_num, data, event.size, portMAX_DELAY);
                    // 触发接收事件
                    // onReceive.trigger(len, data);
                    onReceive.trigger(event);
                    break;
                case UART_BUFFER_FULL:
                    // UART RX缓冲区满
                    onBufferFull.trigger(event);
                    break;
                case UART_FIFO_OVF:
                    // FIFO溢出
                    onFifoOverflow.trigger(event);
                    break;
                default:
                    // 默认事件处理
                    onDefault.trigger(event);
                    break;
                }
            }
        }
    }

public:
    UARTDriver(uart_port_t uart_num, int tx_pin, int rx_pin, int buffer_size, const uart_config_t &uart_config)
        : uart_num(uart_num), tx_pin(tx_pin), rx_pin(rx_pin), buffer_size(buffer_size), uart_config(uart_config)
    {
    }
    UARTDriver(uart_port_t uart_num = UART_NUM_1, int baud = 115200, int buffer_size = 1024)
        : uart_num(uart_num), tx_pin(TX_PIN), rx_pin(RX_PIN), buffer_size(buffer_size)
    {
        uart_config = {
            .baud_rate = baud,                     // 波特率
            .data_bits = UART_DATA_8_BITS,         // 数据位
            .parity = UART_PARITY_DISABLE,         // 校验位
            .stop_bits = UART_STOP_BITS_1,         // 停止位
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // 流控
            .source_clk = UART_SCLK_APB,           // 时钟源
        };
    }

    ~UARTDriver()
    {
        End();
    };
    bool Begin(int task_priority = 10, int task_stack_size = 2048)
    {
        // 安装UART驱动
        if (uart_driver_install(uart_num, buffer_size, buffer_size, 20, &uart_queue, 0) != ESP_OK)
        {
            return false;
        }
        // 配置UART参数
        if (uart_param_config(uart_num, &uart_config) != ESP_OK)
        {
            uart_driver_delete(uart_num);
            return false;
        }
        // 设置UART引脚
        if (uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
        {
            uart_driver_delete(uart_num);
            return false;
        }

        // 创建一个任务处理UART事件
        xTaskCreate((TaskFunction_t)(&UARTDriver::uart_event_task), "uart_event_task", task_stack_size, this, task_priority, &task_handle);

        is_begin = true;
        return true;
    }
    void End()
    {
        if (is_begin)
        {
            if (uart_queue != NULL) // 删除队列
            {
                vQueueDelete(uart_queue);
                uart_queue = NULL;
            }
            if (task_handle != NULL) // 删除任务
            {
                vTaskDelete(task_handle);
                task_handle = NULL;
            }
            uart_driver_delete(uart_num); // 删除驱动
            is_begin = false;
        }
    }

    void RegisterDefaultCallback(Event<uart_event_t>::Callback callback)
    {
        onDefault.registerCallback(callback);
    }
    void RegisterReceiveCallback(Event<uart_event_t>::Callback callback)
    {
        onReceive.registerCallback(callback);
    }
    void RegisterBufferFullCallback(Event<uart_event_t>::Callback callback)
    {
        onBufferFull.registerCallback(callback);
    }
    void RegisterFifoOverflowCallback(Event<uart_event_t>::Callback callback)
    {
        onFifoOverflow.registerCallback(callback);
    }

    void Write(uint8_t data)
    {
        uart_write_bytes(uart_num, &data, 1);
    }
    void Writes(const uint8_t *data, size_t size)
    {
        uart_write_bytes(uart_num, data, size);
    }
    void Writes(const uint8_t *data)
    {
        uart_write_bytes(uart_num, data, strlen((const char *)data));
    }
    void Writes(const char *data)
    {
        uart_write_bytes(uart_num, data, strlen(data));
    }
    void Writes(const String &data)
    {
        uart_write_bytes(uart_num, data.c_str(), data.length());
    }

    void Printf(const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        Writes(buffer);
    }

    void Printflong(const char *format, ...)
    {
        va_list args;
        va_start(args, format);

        // 计算所需缓冲区大小
        size_t requiredSize = vsnprintf(nullptr, 0, format, args) + 1; // 包括结尾的 '\0'
        va_end(args);

        if (requiredSize > 0)
        {
            // 动态分配缓冲区
            char *buffer = (char *)malloc(requiredSize);
            if (buffer)
            {
                // 重新初始化 va_list 并格式化字符串
                va_start(args, format);
                vsnprintf(buffer, requiredSize, format, args);
                va_end(args);

                // 发送格式化后的字符串
                Writes(buffer);

                // 释放分配的内存
                free(buffer);
            }
            else
            {
                // 内存分配失败时的处理（可选）
                Writes("内存分配失败\n");
            }
        }
    }

    uint8_t Read()
    {
        uint8_t data;
        uart_read_bytes(uart_num, &data, 1, portMAX_DELAY);
        return data;
    }
    size_t Reads(uint8_t *data, size_t size)
    {
        return uart_read_bytes(uart_num, data, size, portMAX_DELAY);
    }
    size_t Reads(char *data, size_t size)
    {
        return uart_read_bytes(uart_num, (uint8_t *)data, size, portMAX_DELAY);
    }
    size_t Reads(String &data, size_t size)
    {
        char buffer[size];
        size_t len = Reads(buffer, size);
        data = String(buffer);
        return len;
    }

    void Flush()
    {
        uart_flush(uart_num);
    }
};

#endif // UART_DRIVER_H


/*
uartDriver.RegisterReceiveCallback(
      [](uart_event_t event)
      {
        if (uartDriver.Read() != START_FLAG)
          return;

        uint8_t msgType = uartDriver.Read();
        if (msgType != 0x01)
          return;

        uint16_t dataLen = uartDriver.Read() << 8 | uartDriver.Read();

        size_t rDataLen = uartDriver.Reads(dataTemp, dataLen);
        // uartDriver.Printflong("Data: %s\n", dataTemp);
        pb_istream_t istream = pb_istream_from_buffer(dataTemp, dataLen);
        if (!pb_decode(&istream, Hardware_fields, &hardware))
        {
          uartDriver.Writes("Decoding failed!\n");
          return;
        }

        // static char cpu_name_buffer[MAX_STRING_SIZE] = {0};
        // static char gpu_name_buffer[MAX_STRING_SIZE] = {0};
        // hardware.CpuName.funcs.decode = decode_string;
        // hardware.CpuName.arg = cpu_name_buffer; // 绑定CPU名称缓冲区

        // hardware.GpuName.funcs.decode = decode_string;
        // hardware.GpuName.arg = gpu_name_buffer; // 绑定GPU名称缓冲区
        // uartDriver.Printf("CpuName: %s, GpuName: %s\n", cpu_name_buffer, gpu_name_buffer);
        uartDriver.Printf("CpuName: %s\nGpuName: %s \n", (char *)hardware.CpuName, (char *)hardware.GpuName);
        // uartDriver.Writes("ok");
      });
*/