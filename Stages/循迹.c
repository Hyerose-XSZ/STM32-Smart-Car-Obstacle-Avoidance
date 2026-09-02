#include "stm32f10x.h"

// =========================================================
// 运行参数配置
// =========================================================
#define SPEED_BASE          500    // 直行基准速度 (0~1000)
#define SPEED_HIGH          650    // 转向外侧轮加速速度
#define SPEED_LOW           150    // 转向内侧轮降速速度
#define SPEED_SPIN          600    // 急弯原地差速自转速度

// 循迹探头状态 (0: 白地, 1: 压黑线)
volatile unsigned char track_l2 = 0; // 最左 PC0 (接 IN4)
volatile unsigned char track_l1 = 0; // 中左 PC1 (接 IN3)
volatile unsigned char track_r1 = 0; // 中右 PC2 (接 IN2)
volatile unsigned char track_r2 = 0; // 最右 PC3 (接 IN1)


// =========================================================
// 1. 基础延时
// =========================================================
void Delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 7200; j++);
}


// =========================================================
// 2. 电机驱动与 TIM3 PWM 初始化
// =========================================================
void Motor_Init(void) {
    RCC->APB2ENR |= (1 << 2); // 开启 GPIOA
    RCC->APB1ENR |= (1 << 1); // 开启 TIM3

    // PA0,1,4,5 通用推挽输出, PA6,7 复用推挽输出 (TIM3_CH1/CH2 PWM)
    GPIOA->CRL &= 0x0000FF00;
    GPIOA->CRL |= 0xBB330033;
    GPIOA->BRR = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5);

    TIM3->PSC = 71;           // 预分频 72 -> 1MHz
    TIM3->ARR = 1000 - 1;     // 1kHz PWM
    TIM3->CCMR1 |= (0x6 << 4) | (1 << 3) | (0x6 << 12) | (1 << 11);
    TIM3->CCER  |= (1 << 0) | (1 << 4);
    TIM3->CR1   |= (1 << 0);
}

void Motor_SetSpeed(int speedLeft, int speedRight) {
    // 左轮控制
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

    // 右轮控制
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
// 3. 4路红外循迹模块初始化与差速状态机 (PC0~PC3)
// =========================================================
void Tracking_Init(void) {
    RCC->APB2ENR |= (1 << 4); // 开启 GPIOC

    // PC0~PC3 配置为上拉输入 (MODE=00, CNF=10 -> 0x8)
    GPIOC->CRL &= 0xFFFF0000;
    GPIOC->CRL |= 0x00008888;
    GPIOC->BSRR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
}

void Tracking_Scan(void) {
    // 遇到黑线吸光不导通，模块指示灯灭，引脚输出高电平 (1)
    track_l2 = ((GPIOC->IDR & (1 << 0)) != 0) ? 1 : 0; // IN4 (最左)
    track_l1 = ((GPIOC->IDR & (1 << 1)) != 0) ? 1 : 0; // IN3 (中左)
    track_r1 = ((GPIOC->IDR & (1 << 2)) != 0) ? 1 : 0; // IN2 (中右)
    track_r2 = ((GPIOC->IDR & (1 << 3)) != 0) ? 1 : 0; // IN1 (最右)
}

void Tracking_Drive(void) {
    unsigned char sensor_state = (track_l2 << 3) | (track_l1 << 2) | (track_r1 << 1) | track_r2;

    switch (sensor_state) {
        // 1. 直行：中间两路压线，或中间单路微晃，或十字路口全黑
        case 0b0110:
        case 0b0100:
        case 0b0010:
        case 0b1111:
            Motor_SetSpeed(SPEED_BASE, SPEED_BASE);
            break;

        // 2. 边缘轻度偏航：中度单边转向修正
        case 0b0001: // 最右探头压线 -> 向右修正
            Motor_SetSpeed(SPEED_HIGH, SPEED_LOW);
            break;
        case 0b1000: // 最左探头压线 -> 向左修正
            Motor_SetSpeed(SPEED_LOW, SPEED_HIGH);
            break;

        // 3. 急弯 / 严重偏航：反向差速原地旋转拉回
        case 0b0011: // 右侧两路全黑 -> 强力向右打方向
        case 0b0111:
            Motor_SetSpeed(SPEED_SPIN, -SPEED_SPIN / 3);
            break;
        case 0b1100: // 左侧两路全黑 -> 强力向左打方向
        case 0b1110:
            Motor_SetSpeed(-SPEED_SPIN / 3, SPEED_SPIN);
            break;

        // 4. 全部脱线 (0b0000)：减速向前滑行找线
        case 0b0000:
            Motor_SetSpeed(SPEED_BASE - 100, SPEED_BASE - 100);
            break;

        default:
            Motor_SetSpeed(SPEED_BASE, SPEED_BASE);
            break;
    }
}


// =========================================================
// 4. 主程序：纯循迹调试
// =========================================================
int main(void) {
    Delay_ms(500);         // 延时等待硬件供电稳定
    Motor_Init();          // 初始化电机 PWM
    Tracking_Init();       // 初始化 4路循迹引脚

    while (1) {
        Tracking_Scan();   // 读取 4 路探头
        Tracking_Drive();  // 根据电平调整左右轮速度
        Delay_ms(10);      // 10ms 快速响应
    }
}
