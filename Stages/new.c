#include "stm32f10x.h"

// =========================================================
// 1. 速度与陡峭降速参数配置 (所有参数严格保持原样)
// =========================================================
#define SPEED_BASE           350    // 直道极速巡航基准 (0~1000)
#define SPEED_CURVE_MIN       90    // 弯道深度减速下限 (消除入弯切线动量)
#define SPEED_DROP_RATE       55    // 陡峭降速系数 (检测到偏航立即急踩刹车)

// PD 参数 (直线与微弯平顺巡航)
#define KP   42.0f
#define KD   75.0f

static float last_error = 0.0f;     // 初始置 0，彻底杜绝开机单侧锁死
static float current_error = 0.0f;
static uint8_t memory_dir = 1;      // 偏航记忆: 1:偏右需左转, 2:偏左需右转

// =========================================================
// 避障参数配置 (所有参数严格保持原样)
// =========================================================
#define STOP_THRESHOLD       12.0f  // 避障触发安全距离 (cm)

// 避障各阶段速度分配
#define AVOID_SPEED_RUN      250    // 避障巡航与寻线速度
#define AVOID_SPEED_FAST     460    // 绕障外侧推进速度
#define AVOID_SPEED_SLOW      80    // 阶段1内侧轮速
#define AVOID_SPEED_LEFT_IN  150    // 阶段3回切内侧轮速

// 避障各动作阶段执行时间
#define TIME_TURN_RIGHT      220    // 阶段1：向右大角度借道打舵时间 (ms)
#define TIME_FORWARD_PASS    160    // 阶段2：侧向超越障碍直行时间 (ms)
#define TIME_TURN_LEFT       490    // 阶段3：向左平缓回切打舵时间 (ms)
#define TIME_PULL_STRAIGHT   380    // 阶段5：切回黑线后反向拉正时间 (ms)

volatile float current_distance = 100.0f;

// =========================================================
// 2. 定时器 TIM4 与纯软件安全延时
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

void Delay_us(uint16_t us) {
    TIM4->CNT = 0;
    while (TIM4->CNT < us);
}

// =========================================================
// 3. LED0 状态指示 (PE5)、蜂鸣器 (PB8) 与超声波引脚初始化
//    Trig: PE13 (推挽输出)
//    Echo: PE14 (下拉输入)
// =========================================================
void Sensor_Init(void) {
    RCC->APB2ENR |= (1 << 3) | (1 << 6); // GPIOB, GPIOE

    // PB8 (蜂鸣器) 通用推挽输出 50MHz
    GPIOB->CRH &= 0xFFFFFFF0;
    GPIOB->CRH |= 0x00000003;
    GPIOB->BRR  = (1 << 8);  // PB8 拉低，蜂鸣器静音

    // PE5 (LED0) 推挽输出，初始熄灭 (高电平熄灭)
    GPIOE->CRL &= 0xFF0FFFFF;
    GPIOE->CRL |= 0x00300000;
    GPIOE->BSRR = (1 << 5);

    // PE13 (Trig, 输出 0x3), PE14 (Echo, 输入 0x8)
    GPIOE->CRH &= 0xF00FFFFF;
    GPIOE->CRH |= 0x08300000;
    GPIOE->BRR  = (1 << 13); // Trig 默认拉低
    GPIOE->BRR  = (1 << 14); // Echo 下拉
}

// 超声波测距函数 (PE13 Trig, PE14 Echo)
float Get_Distance(void) {
    unsigned int time = 0;

    // 发送 20us 高电平脉冲
    GPIOE->BSRR = (1 << 13);
    Delay_us(20);
    GPIOE->BRR  = (1 << 13);

    // 等待 Echo 引脚拉高
    TIM4->CNT = 0;
    while ((GPIOE->IDR & (1 << 14)) == 0) {
        if (TIM4->CNT > 20000) return 999.0f; // 20ms 超时退出
    }

    // 计算 Echo 高电平维持时间
    TIM4->CNT = 0;
    while ((GPIOE->IDR & (1 << 14)) != 0) {
        if (TIM4->CNT > 25000) return 999.0f; // 25ms 超时退出
    }

    time = TIM4->CNT;
    return (float)time / 58.0f; // 微秒换算为厘米 (us / 58)
}

// =========================================================
// 4. 电机驱动与 TIM3 硬件 1kHz PWM
// =========================================================
void Motor_Init(void) {
    RCC->APB2ENR |= (1 << 2); // GPIOA
    RCC->APB1ENR |= (1 << 1); // TIM3

    // PA0, PA1, PA4, PA5 通用推挽输出; PA6, PA7 复用推挽输出
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
// 5. 4路循迹初始化与偏差映射 (恢复原版，不带停车)
// =========================================================
void Tracking_Init(void) {
    RCC->APB2ENR |= (1 << 4); // GPIOC

    // PC0~PC3 上拉输入
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

        case 0b1111: return 0.0f; // 十字线正常前行

        // 全白脱轨
        case 0b0000: return 99.0f;

        default: return 0.0f;
    }
}

// =========================================================
// 6. 核心闭环：断崖降速 + 阶梯配合轻度/强力反转
// =========================================================
void PID_Tracking_Loop(void) {
    float raw_error = Tracking_GetError();
    float pid_output = 0.0f;
    float derivative = 0.0f;
    float abs_error = 0.0f;
    int current_base = 0;
    int left_motor = 0;
    int right_motor = 0;

    // 1. 全白脱线自救
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

    // 2. 陡峭自适应降速
    abs_error = (current_error >= 0.0f) ? current_error : -current_error;
    current_base = SPEED_BASE - (int)(abs_error * SPEED_DROP_RATE);
    if (current_base < SPEED_CURVE_MIN) {
        current_base = SPEED_CURVE_MIN;
    }

    // 3. PD 控制计算
    derivative = current_error - last_error;
    pid_output = (KP * current_error) + (KD * derivative);
    last_error = current_error;

    // 4. 阶梯力矩分配
    // 【工况 A：急弯/大弧线边缘 (|Error| >= 4.0)】
    if (abs_error >= 4.0f) {
        if (current_error > 0.0f) {
            left_motor  = 380;
            right_motor = -180; // 右急转
        } else {
            left_motor  = -180; // 左急转
            right_motor = 380;
        }
    }
    // 【工况 B：中度弯道 (|Error| >= 2.0)】
    else if (abs_error >= 2.0f) {
        if (current_error > 0.0f) {
            left_motor  = current_base + 120;
            right_motor = -90;
        } else {
            left_motor  = -90;
            right_motor = current_base + 120;
        }
    }
    // 【工况 C：直道与微弯 (|Error| < 2.0)】
    else {
        left_motor  = current_base + (int)pid_output;
        right_motor = current_base - (int)pid_output;
    }

    // 5. 写入底层电机
    Motor_SetSpeed(left_motor, right_motor);
}

// =========================================================
// 7. 避障动作状态机 (参数完全保持原样)
// =========================================================
void Avoid_Obstacle(void) {
    unsigned int timeout = 0;
    unsigned char hit_line_cnt = 0;

    GPIOE->BRR = (1 << 5); // 点亮 LED0 提示避障中

    // 阶段 0：刹车停顿
    Motor_SetSpeed(0, 0);
    Delay_ms(150);

    Motor_SetSpeed(-450, -450);
    Delay_ms(150);

    Motor_SetSpeed(0, 0);
    Delay_ms(100);

    // 阶段 1：向右借道变道
    Motor_SetSpeed(AVOID_SPEED_FAST, AVOID_SPEED_SLOW);
    Delay_ms(TIME_TURN_RIGHT);

    // 阶段 2：侧向超越障碍物
    Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
    Delay_ms(TIME_FORWARD_PASS);

    // 阶段 3：向左平缓回切打舵
    Motor_SetSpeed(AVOID_SPEED_LEFT_IN, AVOID_SPEED_FAST);
    Delay_ms(TIME_TURN_LEFT);

    // 阶段 4：斜向搜线
    Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
    while (timeout < 300) {
        uint8_t l2 = (GPIOC->IDR & (1 << 0)) ? 1 : 0;
        uint8_t l1 = (GPIOC->IDR & (1 << 1)) ? 1 : 0;
        uint8_t r1 = (GPIOC->IDR & (1 << 2)) ? 1 : 0;
        uint8_t r2 = (GPIOC->IDR & (1 << 3)) ? 1 : 0;

        if (l2 || l1 || r1 || r2) {
            if (++hit_line_cnt >= 2) break;
        } else {
            hit_line_cnt = 0;
        }

        Delay_ms(10);
        timeout++;
    }

    // 阶段 5：顺势右打舵将车身拉平
    Motor_SetSpeed(260, 80);
    Delay_ms(TIME_PULL_STRAIGHT);

    // 阶段 6：重置状态平稳接管原有 PID
    last_error = 0.0f;
    current_error = 0.0f;
    memory_dir = 2;
    GPIOE->BSRR = (1 << 5); // 熄灭 LED0
}

// =========================================================
// 8. 主程序
// =========================================================
int main(void) {
    uint8_t dist_cycle_cnt = 0;
    uint8_t i = 0;

    Sensor_Init();
    Timer4_Init();
    Motor_Init();
    Tracking_Init();

    // 确保电机初始状态绝对抱死静止
    Motor_SetSpeed(0, 0);

    // 通电后等待 5 秒倒计时：LED0 每秒慢闪 1 次 (亮500ms, 灭500ms)，共循环 5 次
    for (i = 0; i < 5; i++) {
        GPIOE->BRR = (1 << 5);  // 点亮 LED0
        Delay_ms(500);
        GPIOE->BSRR = (1 << 5); // 熄灭 LED0
        Delay_ms(500);
    }

    // 启动前快速双闪指示即将发车
    GPIOE->BRR = (1 << 5);  Delay_ms(100);
    GPIOE->BSRR = (1 << 5); Delay_ms(100);
    GPIOE->BRR = (1 << 5);  Delay_ms(100);
    GPIOE->BSRR = (1 << 5); Delay_ms(200);

    while (1) {
        // 超声波分频轮询：约 30~40ms 采样一次，保证循迹 2ms 微分步频不被打乱
        if (++dist_cycle_cnt >= 10) {
            dist_cycle_cnt = 0;
            current_distance = Get_Distance();
        }

        // 障碍物判定：小于等于 12cm 进入避障，否则执行原版闭环循迹
        if (current_distance > 0.0f && current_distance <= STOP_THRESHOLD) {
            Avoid_Obstacle();
            current_distance = 100.0f;
        } else {
            PID_Tracking_Loop();
        }

        Delay_ms(2); // 保持 2ms 稳定微分步频
    }
}
