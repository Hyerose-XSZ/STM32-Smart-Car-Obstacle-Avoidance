#include "stm32f10x.h"

// 软件延时函数 (约 1 毫秒)
void Delay_ms(unsigned int ms) {
    unsigned int i, j;
    for(i = 0; i < ms; i++)
        for(j = 0; j < 7200; j++);
}

// 初始化 PA0, PA1, PA4, PA5 为推挽输出 (50MHz)
void Motor_GPIO_Init(void) {
    // 1. 开启 GPIOA 时钟
    RCC->APB2ENR |= (1 << 2);

    // 2. 配置 PA0, PA1, PA4, PA5
    // CRL 寄存器控制 Pin0 ~ Pin7: 每 4 位控制 1 个引脚 (0x3 代表通用推挽输出 50MHz)
    GPIOA->CRL &= 0xFF00FF00;  // 清空 PA0, PA1, PA4, PA5 的配置位
    GPIOA->CRL |= 0x00330033;  // 设置 PA0, PA1, PA4, PA5 为推挽输出 50MHz

    // 初始状态全部拉低 (停止)
    GPIOA->BRR = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5);
}

// 前进 (正转: IN1=1, IN2=0, IN3=0, IN4=1)
void Motor_Forward(void) {
    GPIOA->BSRR = (1 << 0);  // PA0 (IN1) = 1
    GPIOA->BRR  = (1 << 1);  // PA1 (IN2) = 0
    GPIOA->BRR  = (1 << 4);  // PA4 (IN3) = 0
    GPIOA->BSRR = (1 << 5);  // PA5 (IN4) = 1
}

// 后退 (反转: IN1=0, IN2=1, IN3=1, IN4=0)
void Motor_Backward(void) {
    GPIOA->BRR  = (1 << 0);  // PA0 (IN1) = 0
    GPIOA->BSRR = (1 << 1);  // PA1 (IN2) = 1
    GPIOA->BSRR = (1 << 4);  // PA4 (IN3) = 1
    GPIOA->BRR  = (1 << 5);  // PA5 (IN4) = 0
}

// 停止 (全部拉低)
void Motor_Stop(void) {
    GPIOA->BRR = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5);
}

int main(void) {
    Delay_ms(500);         // 延时等待电源稳定
    Motor_GPIO_Init();     // 初始化电机控制引脚

    while (1) {
        Motor_Forward();   // 持续正转前进
    }
}
