#include "stm32f10x.h"

// =========================================================
// 1. 速度与陡峭降速参数配置 (直道450极速，入弯断崖式降速)
// =========================================================
#define SPEED_BASE           450    // 直道极速巡航基准 (0~1000)
#define SPEED_CURVE_MIN       90    // 弯道深度减速下限 (消除入弯切线动量)
#define SPEED_DROP_RATE       50    // 陡峭降速系数 (检测到偏航立即急踩刹车)

// PD 参数 (直线与微弯平顺巡航)
#define KP   42.0f
#define KD   75.0f

static float last_error = 0.0f;     // 初始置 0，彻底杜绝开机单侧锁死
static float current_error = 0.0f;
static uint8_t memory_dir = 1;      // 偏航记忆: 1:偏右需左转, 2:偏左需右转

// =========================================================
// 避障参数配置 (200速度基准，拉大横向与纵向安全距离)
// =========================================================
#define STOP_THRESHOLD       12.0f  // 避障触发安全距离 (cm)

// 避障各阶段速度分配
#define AVOID_SPEED_RUN      300    // 避障巡航与寻线速度
#define AVOID_SPEED_FAST     460    // 绕障外侧推进速度
#define AVOID_SPEED_SLOW      80    // 阶段1内侧轮速 (形成大差速拉开横向间距)
#define AVOID_SPEED_LEFT_IN  250    // 阶段3回切内侧轮速

// 避障各动作阶段执行时间
#define TIME_TURN_RIGHT      320    // 阶段1：向右大角度借道打舵时间 (ms)
#define TIME_FORWARD_PASS    460    // 阶段2：侧向超越障碍直行时间 (ms)
#define TIME_TURN_LEFT       430    // 阶段3：向左平缓回切打舵时间 (ms)
#define TIME_PULL_STRAIGHT   180    // 阶段5：切回黑线后反向拉正时间 (ms)

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
    RCC->APB1ENR |= (1 << 2); // 使能 TIM4 时钟
    TIM4->PSC = 71;           // 预分频 71 -> 72MHz / 72 = 1MHz (1us/Tick)
    TIM4->ARR = 0xFFFF;
    TIM4->CR1 |= (1 << 0);    // 启动 TIM4
}

void Delay_us(uint16_t us) {
    TIM4->CNT = 0;
    while (TIM4->CNT < us);
}

// =========================================================
// 3. LED0 状态指示 (PE5)、蜂鸣器 (PB8) 与超声波引脚初始化
// =========================================================
void Sensor_Init(void) {
    RCC->APB2ENR |= (1 << 3) | (1 << 6); // 使能 GPIOB, GPIOE 时钟

    // PB8(蜂鸣器推挽输出), PB10(Trig推挽输出), PB11(Echo下拉输入)
    GPIOB->CRH &= 0xFFFF00F0;
    GPIOB->CRH |= 0x00008303;
    GPIOB->BRR  = (1 << 8);  // PB8 拉低，蜂鸣器静音
    GPIOB->BRR  = (1 << 10); // PB10 Trig 初始拉低
    GPIOB->BRR  = (1 << 11); // PB11 初始下拉

    // PE5 (LED0) 推挽输出，初始熄灭 (高电平熄灭)
    GPIOE->CRL &= 0xFF0FFFFF;
    GPIOE->CRL |= 0x00300000;
    GPIOE->BSRR = (1 << 5);
}

// 超声波测距函数 (PB10 Trig, PB11 Echo)
float Get_Distance(void) {
    unsigned int time = 0;

    // 发送 20us 高电平脉冲触发测距
    GPIOB->BSRR = (1 << 10);
    Delay_us(20);
    GPIOB->BRR  = (1 << 10);

    // 等待 Echo 高电平到来 (超时退出保护 20ms)
    TIM4->CNT = 0;
    while ((GPIOB->IDR & (1 << 11)) == 0) {
        if (TIM4->CNT > 20000) return 999.0f;
    }

    // 捕获 Echo 高电平持续时间 (超时退出保护 25ms)
    TIM4->CNT = 0;
    while ((GPIOB->IDR & (1 << 11)) != 0) {
        if (TIM4->CNT > 25000) return 999.0f;
    }

    time = TIM4->CNT;
    return (float)time / 58.0f; // 微秒时间换算为厘米距离
}

// =========================================================
// 4. 电机驱动与 TIM3 硬件 1kHz PWM
// =========================================================
void Motor_Init(void) {
    RCC->APB2ENR |= (1 << 2); // 使能 GPIOA 时钟
    RCC->APB1ENR |= (1 << 1); // 使能 TIM3 时钟

    // PA0, PA1, PA4, PA5 通用推挽输出; PA6, PA7 复用推挽输出 (TIM3_CH1, CH2)
    GPIOA->CRL &= 0x0000FF00;
    GPIOA->CRL |= 0xBB330033;
    GPIOA->BRR = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5);

    TIM3->PSC = 71;           // 72MHz / 72 = 1MHz
    TIM3->ARR = 1000 - 1;     // 1kHz PWM 周期
    TIM3->CCMR1 |= (0x6 << 4) | (1 << 3) | (0x6 << 12) | (1 << 11); // CH1, CH2 PWM模式1与预装载
    TIM3->CCER  |= (1 << 0) | (1 << 4);                              // 使能 CH1, CH2 输出
    TIM3->CR1   |= (1 << 0);                                         // 启动定时器 TIM3
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
        GPIOA->BRR = (1 << 0) | (1 << 1); // 刹车抱死
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
        GPIOA->BRR = (1 << 4) | (1 << 5); // 刹车抱死
        TIM3->CCR2 = 0;
    }
}

// =========================================================
// 5. 4路红外循迹初始化与严格极性偏差映射
// =========================================================
void Tracking_Init(void) {
    RCC->APB2ENR |= (1 << 4); // 使能 GPIOC 时钟

    // PC0~PC3 上拉输入
    GPIOC->CRL &= 0xFFFF0000;
    GPIOC->CRL |= 0x00008888;
    GPIOC->BSRR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
}

// 循迹偏差映射函数
float Tracking_GetError(void) {
    uint8_t l2 = (GPIOC->IDR & (1 << 0)) ? 1 : 0; // IN4 (最左外侧)
    uint8_t l1 = (GPIOC->IDR & (1 << 1)) ? 1 : 0; // IN3 (中左内侧)
    uint8_t r1 = (GPIOC->IDR & (1 << 2)) ? 1 : 0; // IN2 (中右内侧)
    uint8_t r2 = (GPIOC->IDR & (1 << 3)) ? 1 : 0; // IN1 (最右外侧)

    uint8_t state = (l2 << 3) | (l1 << 2) | (r1 << 1) | r2;

    switch (state) {
        case 0b0110: return 0.0f; // 正中居中巡航

        // 车身偏左工况 (黑线在右侧，需向右转 -> 偏差为正)
        case 0b0010: memory_dir = 2; return  1.0f; // 微偏左
        case 0b0011: memory_dir = 2; return  2.5f; // 中度偏左
        case 0b0001:
        case 0b0111: memory_dir = 2; return  4.2f; // 急弯极度偏左

        // 车身偏右工况 (黑线在左侧，需向左转 -> 偏差为负)
        case 0b0100: memory_dir = 1; return -1.0f; // 微偏右
        case 0b1100: memory_dir = 1; return -2.5f; // 中度偏右
        case 0b1000:
        case 0b1110: memory_dir = 1; return -4.2f; // 急弯极度偏右

        case 0b1111: return 0.0f; // 十字线或全黑横线，维持直行

        // 全白脱轨失线
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

    // 1. 全白脱线自救：依据历史记忆立即原地大扭矩甩头搜线
    if (raw_error == 99.0f) {
        GPIOE->BRR = (1 << 5); // 点亮 LED0 报警
        if (memory_dir == 2) {
            Motor_SetSpeed(350, -260); // 顺时针自转搜线
        } else {
            Motor_SetSpeed(-260, 350); // 逆时针自转搜线
        }
        return;
    }

    GPIOE->BSRR = (1 << 5); // 抓到线熄灭 LED0
    current_error = raw_error;

    // 2. 陡峭自适应降速：一旦检测到偏航，巡航速度迅速跌破 200，进急弯压至最低 90
    abs_error = (current_error >= 0.0f) ? current_error : -current_error;
    current_base = SPEED_BASE - (int)(abs_error * SPEED_DROP_RATE);
    if (current_base < SPEED_CURVE_MIN) {
        current_base = SPEED_CURVE_MIN;
    }

    // 3. PD 控制计算
    derivative = current_error - last_error;
    pid_output = (KP * current_error) + (KD * derivative);
    last_error = current_error;

    // 4. 阶梯式转向与主动反转介入
    // -------------------------------------------------------------
    // 【工况 A：急弯/大弧线边缘 (|Error| >= 4.0)】
    // 外侧轮输出 380 推进，内侧轮施加 -180 明确反转，强硬拽回车头
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
    // 降速巡航，且内侧轮介入 -90 轻度反转阻尼，防止切线惯性冲刺
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
    // 高速平稳巡航，纯正向微调差速，绝不反转，杜绝左右蛇形晃荡
    else {
        left_motor  = current_base + (int)pid_output;
        right_motor = current_base - (int)pid_output;
    }

    // 5. 写入底层电机
    Motor_SetSpeed(left_motor, right_motor);
}

// =========================================================
// 7. 避障动作状态机 (带倒车缓冲 + 大横向安全间距)
// =========================================================
void Avoid_Obstacle(void) {
    unsigned int timeout = 0;
    unsigned char hit_line_cnt = 0;

    GPIOE->BRR = (1 << 5); // 点亮 LED0 提示避障中

    // 阶段 0：刹车停顿消除前冲惯性
    Motor_SetSpeed(0, 0);
    Delay_ms(150);

    // 阶段 0.5：主动强制倒车缓冲，拉开与障碍物的物理距离
    Motor_SetSpeed(-450, -450);
    Delay_ms(150);

    // 阶段 0.8：二次静止制动
    Motor_SetSpeed(0, 0);
    Delay_ms(100);

    // 阶段 1：向右大角度借道变道 (拉大横向净距，防止剐蹭)
    Motor_SetSpeed(AVOID_SPEED_FAST, AVOID_SPEED_SLOW);
    Delay_ms(TIME_TURN_RIGHT);

    // 阶段 2：侧向超越障碍物
    Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
    Delay_ms(590);

    // 阶段 3：向左平缓回切打舵 (前置反抽制动抑制角动量 + 单侧强推)
    Motor_SetSpeed(-300, 200);
    Delay_ms(20);
    Motor_SetSpeed(0, 460);
    Delay_ms(TIME_TURN_LEFT);

    // 阶段 4：斜向小角度巡线判定撞线 (最长检测 3 秒)
    Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
    while (timeout < 300) {
        uint8_t l2 = (GPIOC->IDR & (1 << 0)) ? 1 : 0;
        uint8_t l1 = (GPIOC->IDR & (1 << 1)) ? 1 : 0;
        uint8_t r1 = (GPIOC->IDR & (1 << 2)) ? 1 : 0;
        uint8_t r2 = (GPIOC->IDR & (1 << 3)) ? 1 : 0;

        // 任意探头连续 2 次读到黑线判定成功切回
        if (l2 || l1 || r1 || r2) {
            if (++hit_line_cnt >= 2) break;
        } else {
            hit_line_cnt = 0;
        }

        Delay_ms(10);
        timeout++;
    }

    // 阶段 5：顺势右打舵将车身冲顺跑道切线方向
    Motor_SetSpeed(260, 80);
    Delay_ms(TIME_PULL_STRAIGHT);

    // 阶段 6：重置误差与历史记忆，平稳接管原有 PID
    last_error = 0.0f;
    current_error = 0.0f;
    memory_dir = 2; // 设定记忆方向为微向右，防止刚接管时向左反扑
    GPIOE->BSRR = (1 << 5); // 熄灭 LED0
}

// =========================================================
// 8. 主程序
// =========================================================
int main(void) {
    uint8_t dist_cycle_cnt = 0;

    Sensor_Init();
    Timer4_Init();
    Motor_Init();
    Tracking_Init();

    // 开机自检：LED0 快速闪烁 2 次后平顺发车
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
            current_distance = 100.0f; // 避障完成后复位距离标记
        } else {
            PID_Tracking_Loop();
        }

        Delay_ms(2); // 保持 2ms 稳定微分步频
    }
}
