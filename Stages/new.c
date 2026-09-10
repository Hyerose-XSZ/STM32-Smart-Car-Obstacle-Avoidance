#include "stm32f10x.h"

// =========================================================
// 引脚分配说明：
// Trig  -> PE13 (通用推挽输出, 50MHz)
// Echo  -> PE14 (下拉输入)
// LED0  -> PE5  (通用推挽输出, 低电平点亮)
// 定时基准 -> TIM4 (1MHz, 1 Tick = 1us)
// 逻辑规则：前方有障碍物 (距离 <= 15cm) 时点亮 LED0，无障碍物时熄灭
// =========================================================

#define OBSTACLE_DISTANCE  15.0f // 触发门槛 (cm)

// 纯软件毫秒延时
void Delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 7200; j++);
}

// TIM4 初始化：提供 1us 精准微秒计数与延时
void Timer4_Init(void) {
    RCC->APB1ENR |= (1 << 2); // 使能 TIM4 时钟
    TIM4->PSC = 71;           // 72MHz / 72 = 1MHz (1us/Tick)
    TIM4->ARR = 0xFFFF;       // 最大重装载值
    TIM4->CR1 |= (1 << 0);    // 启动计数器
}

// 微秒延时
void Delay_us(uint16_t us) {
    TIM4->CNT = 0;
    while (TIM4->CNT < us);
}

// 端口初始化：配置 PE5(LED0)、PE13(Trig)、PE14(Echo)
void Ultrasonic_LED_Init(void) {
    RCC->APB2ENR |= (1 << 6); // 开启 GPIOE 时钟

    // 1. 配置 PE5 (LED0): 通用推挽输出 (50MHz)
    // 对应 CRL 位段 [23:20] -> 0x3
    GPIOE->CRL &= 0xFF0FFFFF;
    GPIOE->CRL |= 0x00300000;
    GPIOE->BSRR = (1 << 5); // 初始熄灭 (低电平点亮)

    // 2. 配置 PE13 (Trig) 与 PE14 (Echo)
    // 对应 CRH 寄存器:
    // PE13 位段 [23:20] -> 通用推挽输出 50MHz (MODE=11, CNF=00 -> 0x3)
    // PE14 位段 [27:24] -> 下拉输入 (MODE=00, CNF=10 -> 0x8)
    GPIOE->CRH &= 0xF00FFFFF;
    GPIOE->CRH |= 0x08300000;

    // 清零 Trig 引脚电平，Echo 内部下拉保持低电平
    GPIOE->BRR = (1 << 13);
    GPIOE->BRR = (1 << 14); // ODR 清零使 CNF=10 表现为下拉输入
}

// 超声波测距函数 (单位: cm)
float Get_Distance_PE(void) {
    unsigned int time = 0;

    // 产生至少 10us (此处取 20us) 的高电平脉冲启动发射
    GPIOE->BSRR = (1 << 13);
    Delay_us(20);
    GPIOE->BRR  = (1 << 13);

    // 等待 Echo (PE14) 变为高电平，加入超时防止死等
    TIM4->CNT = 0;
    while ((GPIOE->IDR & (1 << 14)) == 0) {
        if (TIM4->CNT > 20000) return 999.0f; // 20ms 超时无响应
    }

    // 测量 Echo (PE14) 高电平维持时间
    TIM4->CNT = 0;
    while ((GPIOE->IDR & (1 << 14)) != 0) {
        if (TIM4->CNT > 25000) return 999.0f; // 超出最大有效测距范围
    }

    time = TIM4->CNT; // 获取微秒脉冲宽度
    return (float)time / 58.0f; // 声速换算: 距离(cm) = 时间(us) / 58
}

int main(void) {
    float distance = 0.0f;

    Timer4_Init();
    Ultrasonic_LED_Init();

    // 开机自检：LED0 快速双闪，提示程序成功跑起
    GPIOE->BRR  = (1 << 5); Delay_ms(100);
    GPIOE->BSRR = (1 << 5); Delay_ms(100);
    GPIOE->BRR  = (1 << 5); Delay_ms(100);
    GPIOE->BSRR = (1 << 5); Delay_ms(300);

    while (1) {
        distance = Get_Distance_PE();

        // 障碍物检测指示逻辑：
        // 前方小于等于 15cm 视为遇障，点亮 LED0；无障碍或超距则熄灭
        if (distance > 0.0f && distance <= OBSTACLE_DISTANCE) {
            GPIOE->BRR = (1 << 5);  // 点亮 LED0
        } else {
            GPIOE->BSRR = (1 << 5); // 熄灭 LED0
        }

        // HC-SR04 要求两次发射间隔至少 60ms，防止上次余波干扰
        Delay_ms(60);
    }
}
