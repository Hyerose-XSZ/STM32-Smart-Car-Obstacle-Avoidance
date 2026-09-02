#include "stm32f10x.h"

// =========================================================
// 绕障核心参数（行进速度降为 300 适配版）
// =========================================================
#define STOP_THRESHOLD        12.0f   // 触发避障距离 (cm)

// 动作速度 (统一以 300 为基准)
#define AVOID_SPEED_RUN       300     // 行进与巡线冲锋速度 (改为 300)
#define AVOID_SPEED_FAST      460     // 转向时外侧轮速度
#define AVOID_SPEED_SLOW      130     // 右转内侧轮速度
#define AVOID_SPEED_LEFT_IN    60     // 左转内侧轮速度 (降低以保证低速下转向力矩充足)

// 动作时间 (单位: ms)
#define TIME_TURN_RIGHT       150     // 阶段1：微幅快速右拐脱轨
#define TIME_FORWARD_PASS     480     // 阶段2：超越障碍物 (速度降到300，时间微增保证超越距离)
#define TIME_TURN_LEFT        380     // 阶段3：强化左转回切 (速度变慢，时间延长以保证充足切回角)
#define TIME_PULL_STRAIGHT    120     // 阶段5：触线顺正拉直时间

// 循迹探头状态 (用于避障后闭环捕获黑线)
volatile unsigned char track_l2 = 0;  // 最左 PC0 (接 IN4)
volatile unsigned char track_l1 = 0;  // 中左 PC1 (接 IN3)
volatile unsigned char track_r1 = 0;  // 中右 PC2 (接 IN2)
volatile unsigned char track_r2 = 0;  // 最右 PC3 (接 IN1)


// =========================================================
// 1. 基础延时与 TIM4 定时器
// =========================================================
void Delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 7200; j++);
}

void Timer4_Init(void) {
    RCC->APB1ENR |= (1 << 2);
    TIM4->PSC = 71;           // 72MHz / 72 = 1MHz (1us/计数)
    TIM4->ARR = 0xFFFF;
    TIM4->CR1 |= (1 << 0);
}

void Delay_us(unsigned int us) {
    TIM4->CNT = 0;
    while (TIM4->CNT < us);
}


// =========================================================
// 2. 超声波与 LED0 初始化 (Trig->PB10, Echo->PB11, LED0->PE5)
// =========================================================
void Sensor_Init(void) {
    RCC->APB2ENR |= (1 << 3) | (1 << 6);

    GPIOB->CRH &= 0xFFFF00F0;
    GPIOB->CRH |= 0x00008303; // PB10推挽输出 50MHz, PB11下拉输入
    GPIOB->BRR  = (1 << 8);   // 静音
    GPIOB->BRR  = (1 << 10);  // Trig 初始低
    GPIOB->BRR  = (1 << 11);  // Echo 下拉有效

    GPIOE->CRL &= 0xFF0FFFFF;
    GPIOE->CRL |= 0x00300000; // PE5 推挽输出
    GPIOE->BSRR = (1 << 5);   // LED0 熄灭
}

float Get_Distance(void) {
    unsigned int time = 0;

    GPIOB->BSRR = (1 << 10);
    Delay_us(20);
    GPIOB->BRR  = (1 << 10);

    TIM4->CNT = 0;
    while ((GPIOB->IDR & (1 << 11)) == 0) {
        if (TIM4->CNT > 20000) return 999.0f;
    }

    TIM4->CNT = 0;
    while ((GPIOB->IDR & (1 << 11)) != 0) {
        if (TIM4->CNT > 25000) return 999.0f;
    }

    time = TIM4->CNT;
    return (float)time / 58.0f;
}


// =========================================================
// 3. 电机驱动与 TIM3 PWM 初始化
// =========================================================
void Motor_Init(void) {
    RCC->APB2ENR |= (1 << 2);
    RCC->APB1ENR |= (1 << 1);

    GPIOA->CRL &= 0x0000FF00;
    GPIOA->CRL |= 0xBB330033;
    GPIOA->BRR = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5);

    TIM3->PSC = 71;
    TIM3->ARR = 1000 - 1;
    TIM3->CCMR1 |= (0x6 << 4) | (1 << 3) | (0x6 << 12) | (1 << 11);
    TIM3->CCER  |= (1 << 0) | (1 << 4);
    TIM3->CR1   |= (1 << 0);
}

void Motor_SetSpeed(int speedLeft, int speedRight) {
    // 左轮
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

    // 右轮
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
// 4. 4路循迹 GPIO 配置与扫描 (PC0~PC3)
// =========================================================
void Tracking_Init(void) {
    RCC->APB2ENR |= (1 << 4);
    GPIOC->CRL &= 0xFFFF0000;
    GPIOC->CRL |= 0x00008888;
    GPIOC->BSRR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
}

void Tracking_Scan(void) {
    track_l2 = ((GPIOC->IDR & (1 << 0)) != 0) ? 1 : 0; // IN4
    track_l1 = ((GPIOC->IDR & (1 << 1)) != 0) ? 1 : 0; // IN3
    track_r1 = ((GPIOC->IDR & (1 << 2)) != 0) ? 1 : 0; // IN2
    track_r2 = ((GPIOC->IDR & (1 << 3)) != 0) ? 1 : 0; // IN1
}


// =========================================================
// 5. 绕障单项测试执行流程
// =========================================================
void Execute_Obstacle_Avoidance(void) {
    unsigned int timeout = 0;
    unsigned char hit_line_cnt = 0;
    unsigned char found_line = 0;

    GPIOE->BRR = (1 << 5); // LED0 点亮表示开始绕障

    // 【步骤 0】刹车缓冲
    Motor_SetSpeed(0, 0);
    Delay_ms(150);

    // 【步骤 1】微幅快速右偏脱轨
    Motor_SetSpeed(AVOID_SPEED_FAST, AVOID_SPEED_SLOW);
    Delay_ms(TIME_TURN_RIGHT);

    // 【步骤 2】稳健超越障碍物 (速度 300)
    Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
    Delay_ms(TIME_FORWARD_PASS);

    // 【步骤 3】左打舵回切 (内侧轮 60，时间 380ms，确保回切角度)
    Motor_SetSpeed(AVOID_SPEED_LEFT_IN, AVOID_SPEED_FAST);
    Delay_ms(TIME_TURN_LEFT);

    // 【步骤 4】以 300 速度切向黑线
    Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
    while (timeout < 300) { // 最长找线 3.0 秒
        Tracking_Scan();

        if (track_l2 || track_l1 || track_r1 || track_r2) {
            hit_line_cnt++;
            if (hit_line_cnt >= 2) { // 连续 2 次采样确认踩黑线
                found_line = 1;
                break;
            }
        } else {
            hit_line_cnt = 0;
        }

        Delay_ms(10);
        timeout++;
    }

    if (found_line) {
        // 【步骤 5】顺线微调：右打舵拉直车身
        Motor_SetSpeed(480, 150);
        Delay_ms(TIME_PULL_STRAIGHT);

        // 成功并入黑线：刹停停稳，LED0 保持常亮
        Motor_SetSpeed(0, 0);
        GPIOE->BRR = (1 << 5);
    } else {
        // 未检测到黑线超时：刹停，LED0 快闪报警
        Motor_SetSpeed(0, 0);
        while (1) {
            GPIOE->BRR = (1 << 5);
            Delay_ms(100);
            GPIOE->BSRR = (1 << 5);
            Delay_ms(100);
        }
    }
}


// =========================================================
// 6. 主程序
// =========================================================
int main(void) {
    float dist = 0.0f;

    Delay_ms(500);
    Timer4_Init();
    Sensor_Init();
    Motor_Init();
    Tracking_Init();

    while (1) {
        dist = Get_Distance();

        if (dist > 0.0f && dist <= STOP_THRESHOLD) {
            Execute_Obstacle_Avoidance();
            while (1); // 测试完成原地锁死，便于检查车身是否正对黑线
        } else {
            // 平时以 300 速度低速直行巡航测试
            Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
        }

        Delay_ms(20);
    }
}
