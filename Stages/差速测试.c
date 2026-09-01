#include "stm32f10x.h"

// 简单毫秒延时
void Delay_ms(unsigned int ms) {
    unsigned int i, j;
    for(i = 0; i < ms; i++)
        for(j = 0; j < 7200; j++);
}

// 1. 初始化 GPIO (PA0, PA1, PA4, PA5 控制方向；PA6, PA7 为 PWM 输出)
void Motor_GPIO_Init(void) {
    // 开启 GPIOA 时钟
    RCC->APB2ENR |= (1 << 2);

    // PA0, PA1, PA4, PA5: 通用推挽输出 50MHz (CNF=00, MODE=11 -> 0x3)
    // PA6, PA7: 复用推挽输出 50MHz (CNF=10, MODE=11 -> 0xB)
    GPIOA->CRL &= 0x0000FF00;
    GPIOA->CRL |= 0xBB330033;

    // 初始状态方向引脚拉低
    GPIOA->BRR = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5);
}

// 2. 初始化 TIM3 通道 1 (PA6) 和通道 2 (PA7) 输出 PWM
void Motor_PWM_Init(void) {
    // 开启 TIM3 时钟
    RCC->APB1ENR |= (1 << 1);

    // 配置预分频器和自动重装载值 (产生 10kHz PWM 信号)
    // 72MHz / (71 + 1) = 1MHz 计数频率，周期 1MHz / 1000 = 1kHz (分辨率 0~1000)
    TIM3->PSC = 71;           // 预分频 72
    TIM3->ARR = 1000 - 1;     // 自动重装载值 1000，方便换算 0%~100% 占空比

    // 配置 TIM3 通道 1 (PA6) 为 PWM 模式 1 (OC1M = 110)，使能预装载 (OC1PE = 1)
    TIM3->CCMR1 |= (0x6 << 4) | (1 << 3);
    // 配置 TIM3 通道 2 (PA7) 为 PWM 模式 1 (OC2M = 110)，使能预装载 (OC2PE = 1)
    TIM3->CCMR1 |= (0x6 << 12) | (1 << 11);

    // 使能通道 1 和通道 2 的输出极性 (高电平有效) 并使能输出
    TIM3->CCER |= (1 << 0) | (1 << 4);

    // 初始化占空比为 0
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;

    // 使能定时器
    TIM3->CR1 |= (1 << 0);
}

// 3. 速度设置接口 (speedLeft / speedRight 取值范围: -1000 到 1000)
// 正数代表该侧正转，负数代表该侧反转，数值绝对值代表占空比 (速度)
void Motor_SetSpeed(int speedLeft, int speedRight) {
    // --- 控制左轮方向与速度 (IN1/IN2, TIM3_CCR1) ---
    if (speedLeft > 0) {
        // 左轮正转
        GPIOA->BSRR = (1 << 0);  // PA0 = 1
        GPIOA->BRR  = (1 << 1);  // PA1 = 0
        TIM3->CCR1  = (speedLeft > 1000) ? 1000 : speedLeft;
    } else if (speedLeft < 0) {
        // 左轮反转
        GPIOA->BRR  = (1 << 0);  // PA0 = 0
        GPIOA->BSRR = (1 << 1);  // PA1 = 1
        TIM3->CCR1  = (-speedLeft > 1000) ? 1000 : -speedLeft;
    } else {
        // 左轮刹车
        GPIOA->BRR = (1 << 0) | (1 << 1);
        TIM3->CCR1 = 0;
    }

    // --- 控制右轮方向与速度 (IN3/IN4, TIM3_CCR2) ---
    if (speedRight > 0) {
        // 右轮正转
        GPIOA->BRR  = (1 << 4);  // PA4 = 0
        GPIOA->BSRR = (1 << 5);  // PA5 = 1
        TIM3->CCR2  = (speedRight > 1000) ? 1000 : speedRight;
    } else if (speedRight < 0) {
        // 右轮反转
        GPIOA->BSRR = (1 << 4);  // PA4 = 1
        GPIOA->BRR  = (1 << 5);  // PA5 = 0
        TIM3->CCR2  = (-speedRight > 1000) ? 1000 : -speedRight;
    } else {
        // 右轮刹车
        GPIOA->BRR = (1 << 4) | (1 << 5);
        TIM3->CCR2 = 0;
    }
}

int main(void) {
    Delay_ms(500);         // 延时等待电源稳定
    Motor_GPIO_Init();     // 初始化 GPIO
    Motor_PWM_Init();      // 初始化 PWM

    while (1) {
        // 1. 直行加速 (左右轮同速: 60% 速度)
        Motor_SetSpeed(600, 600);
        Delay_ms(2000);

        // 2. 弧形大弯左转 (左轮 30% 速度，右轮 70% 速度)
        Motor_SetSpeed(300, 700);
        Delay_ms(2000);

        // 3. 原地快速右转 (左轮正转 50%，右轮反转 50%)
        Motor_SetSpeed(500, -500);
        Delay_ms(1500);

        // 4. 刹车停止
        Motor_SetSpeed(0, 0);
        Delay_ms(2000);
    }
}
