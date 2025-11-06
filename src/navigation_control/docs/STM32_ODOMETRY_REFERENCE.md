# STM32下位机里程计实现参考

## 📋 协议说明

### 数据包格式 (23字节)

```
+--------+----------+----------+-------------+------+------+------+-----------+--------+--------+
| Byte 0 | Byte 1-4 | Byte 5-8 | Byte 9-12   | 13-16| 17-20| 21-24| 25-28     | 29-30  | Byte 31|
+--------+----------+----------+-------------+------+------+------+-----------+--------+--------+
| 0xA5   | delta_x  | delta_y  | delta_theta | vx   | vy   | wz   | timestamp | crc16  | 0x0D   |
+--------+----------+----------+-------------+------+------+------+-----------+--------+--------+
```

### 字段说明

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| header | uint8_t | - | 固定0xA5 |
| delta_x | float | m | 前进距离增量（机器人坐标系X轴） |
| delta_y | float | m | 侧向距离增量（机器人坐标系Y轴） |
| delta_theta | float | rad | 角度增量 |
| vx | float | m/s | X方向速度（机器人坐标系） |
| vy | float | m/s | Y方向速度（机器人坐标系） |
| wz | float | rad/s | 角速度 |
| timestamp | uint32_t | ms | 时间戳（HAL_GetTick()） |
| crc16 | uint16_t | - | CRC16校验 |
| tail | uint8_t | - | 固定0x0D |

---

## 🔧 STM32实现代码

### 1. 数据结构定义

```c
// odometry.h
#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include <stdint.h>

// 里程计增量数据包
typedef struct __attribute__((packed)) {
    uint8_t header;        // 0xA5
    float delta_x;         // 前进距离增量 (m)
    float delta_y;         // 侧向距离增量 (m)
    float delta_theta;     // 角度增量 (rad)
    float vx;              // X速度 (m/s)
    float vy;              // Y速度 (m/s)
    float wz;              // 角速度 (rad/s)
    uint32_t timestamp;    // 时间戳 (ms)
    uint16_t crc16;        // CRC16校验
    uint8_t tail;          // 0x0D
} OdometryDeltaPacket;

// 三轮全向轮运动学参数
#define WHEEL_RADIUS        0.050f   // 轮子半径 (m) - 根据实际修改
#define WHEEL_BASE          0.200f   // 轮距（中心到轮子距离） (m)
#define ENCODER_PPR         1024     // 编码器每转脉冲数 - 根据实际修改
#define GEAR_RATIO          20.0f    // 减速比 - 根据实际修改
#define SAMPLE_TIME         0.02f    // 采样周期 50Hz = 0.02s

// 函数声明
void Odometry_Init(void);
void Odometry_Update(void);
void Odometry_SendPacket(void);

#endif
```

### 2. 核心实现代码

```c
// odometry.c
#include "odometry.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include <math.h>
#include <string.h>

// CRC16校验表 (与上位机一致)
static const uint16_t CRC16_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  // ... (完整表格省略，需要包含完整256项)
};

#define CRC16_INIT 0xFFFF

// CRC16计算函数
uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t ch_data;
    if (pchMessage == NULL) return 0xFFFF;
    while (dwLength--) {
        ch_data = *pchMessage++;
        wCRC = ((uint16_t)(wCRC) >> 8) ^ CRC16_TABLE[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
    }
    return wCRC;
}

// 上次编码器计数值
static int16_t last_encoder1 = 0;
static int16_t last_encoder2 = 0;
static int16_t last_encoder3 = 0;

// 当前速度
static float current_vx = 0.0f;
static float current_vy = 0.0f;
static float current_wz = 0.0f;

/**
 * @brief 初始化里程计
 */
void Odometry_Init(void)
{
    // 读取初始编码器值
    last_encoder1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);  // 根据实际修改
    last_encoder2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    last_encoder3 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

/**
 * @brief 编码器增量转换为轮子线速度
 * @param delta_ticks 编码器增量 (脉冲)
 * @return 线速度 (m/s)
 */
static float EncoderDeltaToVelocity(int16_t delta_ticks)
{
    // 计算公式: v = (delta_ticks / PPR) * (2π * R / GEAR_RATIO) / dt
    // 简化: v = delta_ticks * K
    // K = 2π * R / (PPR * GEAR_RATIO * dt)
    
    float K = (2.0f * M_PI * WHEEL_RADIUS) / (ENCODER_PPR * GEAR_RATIO * SAMPLE_TIME);
    return delta_ticks * K;
}

/**
 * @brief 三轮全向轮逆运动学（轮速 → 机器人速度）
 * @param v1, v2, v3 三个轮子的线速度 (m/s)
 * @param vx, vy, wz 输出：机器人速度
 * 
 * 轮子布局（俯视图）:
 *       前 (0°)
 *        ↑
 *       轮1
 *      /   \
 *  轮2        轮3
 *  (120°)   (240°)
 * 
 * 运动学方程:
 * v1 = vx * cos(0°)   + vy * sin(0°)   + wz * R = vx + wz*R
 * v2 = vx * cos(120°) + vy * sin(120°) + wz * R = -0.5*vx + 0.866*vy + wz*R
 * v3 = vx * cos(240°) + vy * sin(240°) + wz * R = -0.5*vx - 0.866*vy + wz*R
 * 
 * 逆解:
 * vx = (2*v1 - v2 - v3) / 3
 * vy = (v2 - v3) * sqrt(3) / 3
 * wz = (v1 + v2 + v3) / (3*R)
 */
static void OmniKinematics_Inverse(float v1, float v2, float v3, 
                                   float *vx, float *vy, float *wz)
{
    *vx = (2.0f * v1 - v2 - v3) / 3.0f;
    *vy = (v2 - v3) * 0.57735f;  // sqrt(3)/3 ≈ 0.57735
    *wz = (v1 + v2 + v3) / (3.0f * WHEEL_BASE);
}

/**
 * @brief 更新里程计（在50Hz定时器中调用）
 */
void Odometry_Update(void)
{
    // 1. 读取当前编码器值
    int16_t curr_encoder1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t curr_encoder2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t curr_encoder3 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    
    // 2. 计算增量（处理溢出）
    int16_t delta1 = curr_encoder1 - last_encoder1;
    int16_t delta2 = curr_encoder2 - last_encoder2;
    int16_t delta3 = curr_encoder3 - last_encoder3;
    
    // 更新上次值
    last_encoder1 = curr_encoder1;
    last_encoder2 = curr_encoder2;
    last_encoder3 = curr_encoder3;
    
    // 3. 编码器增量 → 轮子速度
    float v1 = EncoderDeltaToVelocity(delta1);
    float v2 = EncoderDeltaToVelocity(delta2);
    float v3 = EncoderDeltaToVelocity(delta3);
    
    // 4. 轮子速度 → 机器人速度（机器人坐标系）
    OmniKinematics_Inverse(v1, v2, v3, &current_vx, &current_vy, &current_wz);
    
    // 5. 发送数据包
    Odometry_SendPacket();
}

/**
 * @brief 发送里程计数据包
 */
void Odometry_SendPacket(void)
{
    OdometryDeltaPacket packet;
    
    // 1. 组装数据
    packet.header = 0xA5;
    
    // 计算增量（速度 × 时间）
    packet.delta_x = current_vx * SAMPLE_TIME;
    packet.delta_y = current_vy * SAMPLE_TIME;
    packet.delta_theta = current_wz * SAMPLE_TIME;
    
    // 速度
    packet.vx = current_vx;
    packet.vy = current_vy;
    packet.wz = current_wz;
    
    // 时间戳
    packet.timestamp = HAL_GetTick();
    
    // 尾标
    packet.tail = 0x0D;
    
    // 2. 计算CRC16
    packet.crc16 = Get_CRC16_Check_Sum((uint8_t*)&packet, 
                                        sizeof(packet) - 3,  // 不包括crc16和tail
                                        CRC16_INIT);
    
    // 3. 通过UART发送
    HAL_UART_Transmit(&huart1, (uint8_t*)&packet, sizeof(packet), 10);
}
```

### 3. 在main.c中集成

```c
// main.c

// 定时器中断回调 (50Hz)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6) {  // 假设使用TIM6作为50Hz定时器
        Odometry_Update();  // 更新并发送里程计
    }
}

int main(void)
{
    // ... HAL初始化 ...
    
    // 启动编码器定时器
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // 轮1
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  // 轮2
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // 轮3
    
    // 启动50Hz定时器（20ms中断一次）
    HAL_TIM_Base_Start_IT(&htim6);
    
    // 初始化里程计
    Odometry_Init();
    
    while (1) {
        // 主循环
    }
}
```

---

## ⚙️ CubeMX配置

### 定时器配置（编码器模式）

**TIM2, TIM3, TIM4** (编码器输入):
- Mode: `Encoder Mode - TI1 and TI2`
- Counter Period: `65535` (16位最大值)
- Prescaler: `0` (不分频)

**TIM6** (50Hz定时器):
- Prescaler: `7199` (假设72MHz时钟)
- Counter Period: `199`
- 中断频率 = 72MHz / (7200 × 200) = 50Hz

---

## 🧪 测试与调试

### 测试步骤

1. **静止测试**: 机器人静止，delta应该为0
2. **单轮测试**: 手动转动单个轮子，观察速度
3. **前进测试**: 手推机器人前进，vx应为正
4. **旋转测试**: 原地旋转，wz应有值

### 调试工具

```c
// 通过串口打印调试信息
void Odometry_Debug_Print(void)
{
    char buffer[100];
    sprintf(buffer, "Vx=%.3f, Vy=%.3f, Wz=%.3f\r\n", 
            current_vx, current_vy, current_wz);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}
```

---

## 📌 注意事项

### 1. 坐标系定义
- **机器人坐标系**: X轴向前，Y轴向左，Z轴向上
- **增量数据**: 始终在机器人坐标系中
- **上位机负责**: 转换到世界坐标系

### 2. 编码器方向
确保编码器方向正确：
- 轮子正转 → 编码器增加
- 如果方向反了，在代码中取反: `delta1 = -delta1;`

### 3. 单位统一
- **距离**: 米 (m)
- **速度**: 米/秒 (m/s)
- **角度**: 弧度 (rad)
- **时间**: 秒 (s)

### 4. 溢出处理
16位编码器计数器会溢出：
```c
// int16_t自动处理溢出
// 例如: 32767 → 32768 = -32768 (正确差值为1)
```

---

## 🔍 故障排查

| 问题 | 可能原因 | 解决方法 |
|------|---------|---------|
| 上位机收不到数据 | 串口未连接/波特率错误 | 检查硬件连接和配置 |
| CRC校验失败 | CRC算法不一致 | 确保表格完全相同 |
| 速度异常大 | 参数配置错误 | 检查WHEEL_RADIUS, ENCODER_PPR等 |
| 方向相反 | 编码器方向错 | 在代码中取反增量 |
| 数据不稳定 | 定时器频率不稳定 | 检查定时器配置 |

---

## 📚 参考资料

- [三轮全向轮运动学推导](https://blog.csdn.net/xxx)
- [STM32编码器模式配置](https://www.st.com/resource/en/application_note/xxx.pdf)
- ROS2 nav_msgs/Odometry消息格式

---

**如有问题，请联系上位机团队！** 🤝
