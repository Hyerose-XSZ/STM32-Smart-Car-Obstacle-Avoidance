#include "stm32f10x.h"

// =========================================================
// 1. 速度与降速参数配置 (直道提速至360，弯道维持原减速深度)
// =========================================================
#define SPEED_BASE           360    // 直道巡航速度提速 (320 -> 360)
#define SPEED_CURVE_MIN       90    // 弯道深度减速下限保持不变 (确保入弯稳健)
#define SPEED_DROP_RATE       70    // 增大降速梯度 (确保从更高直道速度入弯时刹车依然及时)

// PD 参数 (微调D项以抑制更高直道速度下的惯性摆头)
#define KP   42.0f
#define KD   82.0f

static float last_error = 0.0f;
static float current_error = 0.0f;
static uint8_t memory_dir = 1;      // 偏航记忆: 1:偏右需左转, 2:偏左需右转

// =========================================================
// 2. 定时器 TIM4 与延时函数
// =========================================================
void Delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 7200; j++);
}

void Timer4_Init(void) {
    RCC->APB1ENR |= (1 << 2); // 使能 TIM4
    TIM4->PSC = 71;           // 1MHz (1us/Tick)
    TIM4->ARR = 0xFFFF;
    TIM4->CR1 |= (1 << 0);
}

void Delay_us(unsigned int us) {
    TIM4->CNT = 0;
    while (TIM4->CNT < us);
}

// =========================================================
// 3. LED0 状态指示 (PE5) 与蜂鸣器 (PB8)
// =========================================================
void Sensor_Init(void) {
    RCC->APB2ENR |= (1 << 3) | (1 << 6); // GPIOB, GPIOE

    // PB8 蜂鸣器输出，拉低静音
    GPIOB->CRH &= 0xFFFFFFF0;
    GPIOB->CRH |= 0x00000003;
    GPIOB->BRR  = (1 << 8);

    // PE5 (LED0) 推挽输出，初始熄灭
    GPIOE->CRL &= 0xFF0FFFFF;
    GPIOE->CRL |= 0x00300000;
    GPIOE->BSRR = (1 << 5);
}

// =========================================================
// 4. 电机驱动与 TIM3 硬件 1kHz PWM
// =========================================================
void Motor_Init(void) {
    RCC->APB2ENR |= (1 << 2); // GPIOA
    RCC->APB1ENR |= (1 << 1); // TIM3

    GPIOA->CRL &= 0x0000FF00;
    GPIOA->CRL |= 0xBB330033;
    GPIOA->BRR = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5);

    TIM3->PSC = 71;           // 72MHz / 72 = 1MHz
    TIM3->ARR = 1000 - 1;     // 1kHz PWM
    TIM3->CCMR1 |= (0x6 << 4) | (1 << 3) | (0x6 << 12) | (1 << 11);
    TIM3->CCER  |= (1 << 0) | (1 << 4);
    TIM3->CR1   |= (1 << 0);
}

void Motor_SetSpeed(int speedLeft, int speedRight) {
    // 左轮控速 (IN1: PA0, IN2: PA1, ENA: PA6)
    if (speedLeft > 0) {
        GPIOA->BSRR = (1 << 0); GPIOA->BRR  = (1 << 1);
        TIM3->CCR1  = (speedLeft > 1000) ? 1000 : speedLeft;
    } else if (speedLeft < 0) {
        GPIOA->BRR  = (1 << 0); GPIOA->BSRR = (1 << 1);
        TIM3->CCR1  = (-speedLeft > 1000) ? 1000 : -speedLeft;
    } else {
        GPIOA->BRR = (1 << 0) | (1 << 1);
        TIM3->CCR1 = 0;
    }

    // 右轮控速 (IN3: PA4, IN4: PA5, ENB: PA7)
    if (speedRight > 0) {
        GPIOA->BRR  = (1 << 4); GPIOA->BSRR = (1 << 5);
        TIM3->CCR2  = (speedRight > 1000) ? 1000 : speedRight;
    } else if (speedRight < 0) {
        GPIOA->BSRR = (1 << 4); GPIOA->BRR  = (1 << 5);
        TIM3->CCR2  = (-speedRight > 1000) ? 1000 : -speedRight;
    } else {
        GPIOA->BRR = (1 << 4) | (1 << 5);
        TIM3->CCR2 = 0;
    }
}

// =========================================================
// 5. 4路红外循迹读取与偏差映射
// =========================================================
void Tracking_Init(void) {
    RCC->APB2ENR |= (1 << 4); // 使能 GPIOC

    GPIOC->CRL &= 0xFFFF0000;
    GPIOC->CRL |= 0x00008888;
    GPIOC->BSRR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
}

float Tracking_GetError(void) {
    uint8_t l2 = (GPIOC->IDR & (1 << 0)) ? 1 : 0; // IN4 (最左)
    uint8_t l1 = (GPIOC->IDR & (1 << 1)) ? 1 : 0; // IN3 (中左)
    uint8_t r1 = (GPIOC->IDR & (1 << 2)) ? 1 : 0; // IN2 (中右)
    uint8_t r2 = (GPIOC->IDR & (1 << 3)) ? 1 : 0; // IN1 (最右)

    uint8_t state = (l2 << 3) | (l1 << 2) | (r1 << 1) | r2;

    switch (state) {
        case 0b0110: return 0.0f; // 正中

        // 偏左工况 (需向右转)
        case 0b0010: memory_dir = 2; return  1.0f; // 微偏左
        case 0b0011: memory_dir = 2; return  2.5f; // 中度偏左
        case 0b0001:
        case 0b0111: memory_dir = 2; return  4.2f; // 急弯极度偏左

        // 偏右工况 (需向左转)
        case 0b0100: memory_dir = 1; return -1.0f; // 微偏右
        case 0b1100: memory_dir = 1; return -2.5f; // 中度偏右
        case 0b1000:
        case 0b1110: memory_dir = 1; return -4.2f; // 急弯极度偏右

        case 0b1111: return 0.0f;

        // 全白脱轨
        case 0b0000: return 99.0f;

        default: return 0.0f;
    }
}

// =========================================================
// 6. 核心闭环：直道提速 + 入弯精准深减速 + 反转保持
// =========================================================
void PID_Tracking_Loop(void) {
    float raw_error = Tracking_GetError();
    float pid_output = 0.0f;
    float derivative = 0.0f;
    float abs_error = 0.0f;
    int current_base = 0;
    int left_motor = 0;
    int right_motor = 0;

    // 1. 全白脱轨自救保持不变
    if (raw_error == 99.0f) {
        GPIOE->BRR = (1 << 5); // 点亮 LED0
        if (memory_dir == 2) {
            Motor_SetSpeed(350, -260); // 顺时针自转
        } else {
            Motor_SetSpeed(-260, 350); // 逆时针自转
        }
        return;
    }

    GPIOE->BSRR = (1 << 5); // 熄灭 LED0
    current_error = raw_error;

    // 2. 自适应降速：直道以 360 疾驰，一旦偏航迅速压回 90~150 维持入弯手感
    abs_error = (current_error >= 0.0f) ? current_error : -current_error;
    current_base = SPEED_BASE - (int)(abs_error * SPEED_DROP_RATE);
    if (current_base < SPEED_CURVE_MIN) {
        current_base = SPEED_CURVE_MIN;
    }

    // 3. PD 控制量运算
    derivative = current_error - last_error;
    pid_output = (KP * current_error) + (KD * derivative);
    last_error = current_error;

    // 4. 转向与反力矩分配 (保持原入弯反转力度与姿态)
    // -------------------------------------------------------------
    // 工况 A：急弯/大弧线边缘 (|Error| >= 4.0)
    if (abs_error >= 4.0f) {
        if (current_error > 0.0f) {
            left_motor  = 380;
            right_motor = -180; // 右急转
        } else {
            left_motor  = -180; // 左急转
            right_motor = 380;
        }
    }
    // 工况 B：中度弯道 (|Error| >= 2.0)
    else if (abs_error >= 2.0f) {
        if (current_error > 0.0f) {
            left_motor  = current_base + 120;
            right_motor = -90;  // 轻度反转阻尼
        } else {
            left_motor  = -90;
            right_motor = current_base + 120;
        }
    }
    // 工况 C：直道与微弯 (|Error| < 2.0)
    else {
        left_motor  = current_base + (int)pid_output;
        right_motor = current_base - (int)pid_output;
    }

    // 5. 写入底层电机
    Motor_SetSpeed(left_motor, right_motor);
}

// =========================================================
// 7. 主程序
// =========================================================
int main(void) {
    Sensor_Init();
    Timer4_Init();
    Motor_Init();
    Tracking_Init();

    // 开机自检闪烁
    GPIOE->BRR = (1 << 5);  Delay_ms(100);
    GPIOE->BSRR = (1 << 5); Delay_ms(100);
    GPIOE->BRR = (1 << 5);  Delay_ms(100);
    GPIOE->BSRR = (1 << 5); Delay_ms(200);

    while (1) {
        PID_Tracking_Loop();
        Delay_ms(2);
    }
}
