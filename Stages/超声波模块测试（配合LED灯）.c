#include "stm32f10x.h"

volatile float current_distance = 0.0f;
volatile int   error_code = 0;

void Delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 7200; j++);
}

void Timer4_Init(void) {
    RCC->APB1ENR |= (1 << 2); // 开启 TIM4 时钟
    TIM4->PSC = 71;           // 72MHz / 72 = 1MHz (1us/计数)
    TIM4->ARR = 0xFFFF;
    TIM4->CR1 |= (1 << 0);
}

void Delay_us(unsigned int us) {
    TIM4->CNT = 0;
    while (TIM4->CNT < us);
}

void Diagnostic_Init(void) {
    // 开启 GPIOB 和 GPIOE 时钟
    RCC->APB2ENR |= (1 << 3) | (1 << 6);

    // 1. PB10(Trig): 通用推挽输出 50MHz (0x3) -> bit 8~11
    // 2. PB11(Echo): 下拉输入 (MODE=00, CNF=10 -> 0x8) -> bit 12~15
    GPIOB->CRH &= 0xFFFF00FF;
    GPIOB->CRH |= 0x00008300;
    GPIOB->BRR  = (1 << 10); // Trig 初始拉低
    GPIOB->BRR  = (1 << 11); // Echo 配置为下拉 (ODR对应位写0)

    // 3. PE5 (LED0): 推挽输出
    GPIOE->CRL &= 0xFF0FFFFF;
    GPIOE->CRL |= 0x00300000;
    GPIOE->BSRR = (1 << 5);  // 初始熄灭

    // 4. PB8 (板载蜂鸣器): 保持拉低静音
    GPIOB->CRH &= 0xFFFFFFF0;
    GPIOB->CRH |= 0x00000003;
    GPIOB->BRR  = (1 << 8);
}

float HCSR04_Get_Distance(void) {
    unsigned int time = 0;

    // 1. 发送 20us 高电平脉冲 (PB10)
    GPIOB->BSRR = (1 << 10);
    Delay_us(20);
    GPIOB->BRR  = (1 << 10);

    // 2. 等待 Echo (PB11) 变高 (硬件计时等待最多 30ms)
    TIM4->CNT = 0;
    while ((GPIOB->IDR & (1 << 11)) == 0) {
        if (TIM4->CNT > 30000) {
            error_code = -1; // 超时未收到回波起始
            return -1.0f;
        }
    }

    // 3. 开始计时
    TIM4->CNT = 0;

    // 4. 等待 Echo (PB11) 变低 (最多等待 35ms)
    while ((GPIOB->IDR & (1 << 11)) != 0) {
        if (TIM4->CNT > 35000) {
            error_code = -2; // 超时未结束
            return -2.0f;
        }
    }

    time = TIM4->CNT;
    error_code = 0;
    return (float)time / 58.0f;
}

int main(void) {
    Delay_ms(300);
    Timer4_Init();
    Diagnostic_Init();

    while (1) {
        current_distance = HCSR04_Get_Distance();

        if (error_code != 0) {
            // 异常超时：LED0 快速闪烁
            GPIOE->ODR ^= (1 << 5);
            Delay_ms(150);
        } 
        else if (current_distance > 0.0f && current_distance <= 15.0f) {
            // 正常且在 15cm 内：LED0 常亮
            GPIOE->BRR = (1 << 5);
            Delay_ms(50);
        } 
        else {
            // 正常且前方无障碍：LED0 熄灭
            GPIOE->BSRR = (1 << 5);
            Delay_ms(50);
        }
    }
}
