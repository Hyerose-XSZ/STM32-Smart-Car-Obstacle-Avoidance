#include "stm32f10x.h"

// =========================================================
// 全局变量：Keil Watch 窗口可实时监视
// =========================================================
volatile float current_distance = 0.0f; // 实时测距数值 (单位: cm)


// =========================================================
// 1. 延时与 TIM4 微秒定时器
// =========================================================

void Delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 7200; j++);
}

// 初始化 TIM4：1MHz 计数频率，计 1 次 = 1 微秒
void Timer4_Init(void) {
    RCC->APB1ENR |= (1 << 2); // 开启 TIM4 时钟
    TIM4->PSC = 71;           // 72MHz / 72 = 1MHz
    TIM4->ARR = 0xFFFF;       // 最大重装载值 65535us
    TIM4->CR1 |= (1 << 0);    // 启动 TIM4
}

void Delay_us(unsigned int us) {
    TIM4->CNT = 0;
    while (TIM4->CNT < us);
}


// =========================================================
// 2. 超声波与状态指示灯初始化 (Trig->PB10, Echo->PB11, LED0->PE5)
// =========================================================

void Sensor_Init(void) {
    // 开启 GPIOB 和 GPIOE 时钟
    RCC->APB2ENR |= (1 << 3) | (1 << 6);

    // 配置 PB8(蜂鸣器推挽输出静音), PB10(Trig推挽输出 50MHz), PB11(Echo下拉输入)
    GPIOB->CRH &= 0xFFFF00F0;
    GPIOB->CRH |= 0x00008303;
    GPIOB->BRR  = (1 << 8);   // PB8 拉低，蜂鸣器静音
    GPIOB->BRR  = (1 << 10);  // PB10 (Trig) 初始拉低
    GPIOB->BRR  = (1 << 11);  // PB11 (Echo) 下拉电阻生效

    // PE5 (板载 LED0): 推挽输出，初始熄灭 (高电平灭)
    GPIOE->CRL &= 0xFF0FFFFF;
    GPIOE->CRL |= 0x00300000;
    GPIOE->BSRR = (1 << 5);
}

// 获取前方障碍物距离 (cm)
float Get_Distance(void) {
    unsigned int time = 0;

    // 1. 发送 20us 高电平脉冲
    GPIOB->BSRR = (1 << 10);
    Delay_us(20);
    GPIOB->BRR  = (1 << 10);

    // 2. 等待 Echo (PB11) 变高 (最长等待 30ms)
    TIM4->CNT = 0;
    while ((GPIOB->IDR & (1 << 11)) == 0) {
        if (TIM4->CNT > 30000) return 999.0f;
    }

    // 3. 开始计时
    TIM4->CNT = 0;

    // 4. 等待 Echo (PB11) 变低 (最长等待 35ms)
    while ((GPIOB->IDR & (1 << 11)) != 0) {
        if (TIM4->CNT > 35000) return 999.0f;
    }

    // 5. 换算距离：距离(cm) = 微秒数 / 58.0
    time = TIM4->CNT;
    return (float)time / 58.0f;
}


// =========================================================
// 3. 电机 GPIO 与 TIM3 PWM 配置
// =========================================================

void Motor_Init(void) {
    // 开启 GPIOA 与 TIM3 时钟
    RCC->APB2ENR |= (1 << 2);
    RCC->APB1ENR |= (1 << 1);

    // PA0, PA1, PA4, PA5: 通用推挽输出 50MHz (0x3)
    // PA6, PA7: 复用推挽输出 50MHz (0xB, 用于 TIM3_CH1/CH2 PWM)
    GPIOA->CRL &= 0x0000FF00;
    GPIOA->CRL |= 0xBB330033;

    // 方向引脚初始拉低
    GPIOA->BRR = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5);

    // TIM3 配置：1000 分辨率，1kHz PWM 频率
    TIM3->PSC = 71;           // 预分频 72 -> 1MHz
    TIM3->ARR = 1000 - 1;     // 自动重装载 1000

    // 通道 1 (PA6) 与 通道 2 (PA7) 配置为 PWM 模式 1
    TIM3->CCMR1 |= (0x6 << 4) | (1 << 3) | (0x6 << 12) | (1 << 11);
    TIM3->CCER  |= (1 << 0) | (1 << 4); // 使能输出
    TIM3->CR1   |= (1 << 0);            // 启动定时器
}

// 设置左右轮速度 (范围: -1000 到 1000)
void Motor_SetSpeed(int speedLeft, int speedRight) {
    // --- 控制左轮 (PA0/PA1, TIM3_CH1) ---
    if (speedLeft > 0) {
        GPIOA->BSRR = (1 << 0);
        GPIOA->BRR  = (1 << 1);
        TIM3->CCR1  = (speedLeft > 1000) ? 1000 : speedLeft;
    } else if (speedLeft < 0) {
        GPIOA->BRR  = (1 << 0);
        GPIOA->BSRR = (1 << 1);
        TIM3->CCR1  = (-speedLeft > 1000) ? 1000 : -speedLeft;
    } else {
        GPIOA->BRR = (1 << 0) | (1 << 1);
        TIM3->CCR1 = 0;
    }

    // --- 控制右轮 (PA4/PA5, TIM3_CH2) ---
    if (speedRight > 0) {
        GPIOA->BRR  = (1 << 4);
        GPIOA->BSRR = (1 << 5);
        TIM3->CCR2  = (speedRight > 1000) ? 1000 : speedRight;
    } else if (speedRight < 0) {
        GPIOA->BSRR = (1 << 4);
        GPIOA->BRR  = (1 << 5);
        TIM3->CCR2  = (-speedRight > 1000) ? 1000 : -speedRight;
    } else {
        GPIOA->BRR = (1 << 4) | (1 << 5);
        TIM3->CCR2 = 0;
    }
}


// =========================================================
// 4. 主循环：避障逻辑 (阈值 5.0cm)
// =========================================================

int main(void) {
    Delay_ms(500);         // 延时等待上电供电稳定
    Timer4_Init();         // 初始化 TIM4 微秒定时器
    Sensor_Init();         // 初始化超声波与 LED0
    Motor_Init();          // 初始化电机 GPIO 与 PWM

    while (1) {
        // 读取前方距离
        current_distance = Get_Distance();

        // 避障判断：距离 <= 5.0cm 触发停车
        if (current_distance > 0.0f && current_distance <= 5.0f) {
            Motor_SetSpeed(0, 0);     // 刹停电机
            GPIOE->BRR = (1 << 5);    // 点亮 LED0 指示检测到障碍物
        } else {
            Motor_SetSpeed(450, 450); // 前方安全，以 45% 速度平稳前进
            GPIOE->BSRR = (1 << 5);   // 熄灭 LED0
        }

        // 测距采样周期延时 40ms
        Delay_ms(40);
    }
}
