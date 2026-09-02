#include "stm32f10x.h"

// =========================================================
// 1. 循迹运行参数配置 (巡航速度锁死 300，强化大弯转向力矩)
// =========================================================
#define SPEED_BASE          300    // 直行基准巡航速度 (0~1000)
#define SPEED_HIGH          520    // 弯道外侧轮加速速度
#define SPEED_LOW            80    // 弯道内侧轮速度 (压低以缩小转弯半径，克服大弯冲出)
#define SPEED_SPIN          480    // 急弯原地自转速度

// =========================================================
// 2. 绕障核心参数配置 (验证通过的紧凑参数)
// =========================================================
#define STOP_THRESHOLD      12.0f  // 触发避障距离 (cm)

#define AVOID_SPEED_RUN     300    // 绕障直行与寻线冲锋速度 (锁定 300)
#define AVOID_SPEED_FAST    460    // 转向时外侧轮速度
#define AVOID_SPEED_SLOW    130    // 右转内侧轮速度
#define AVOID_SPEED_LEFT_IN  60    // 左转内侧轮速度

#define TIME_TURN_RIGHT     150    // 阶段1：微幅快速右拐脱轨 (ms)
#define TIME_FORWARD_PASS   480    // 阶段2：斜向超越障碍物 (ms)
#define TIME_TURN_LEFT      380    // 阶段3：强化左转回切 (ms)
#define TIME_PULL_STRAIGHT  120    // 阶段5：触线顺正拉直时间 (ms)

// 状态与记忆变量
volatile float current_distance = 100.0f; // 前方测距值 (cm)
volatile unsigned char track_l2 = 0;      // 最左 PC0 (接 IN4)
volatile unsigned char track_l1 = 0;      // 中左 PC1 (接 IN3)
volatile unsigned char track_r1 = 0;      // 中右 PC2 (接 IN2)
volatile unsigned char track_r2 = 0;      // 最右 PC3 (接 IN1)

// 偏航历史记忆: 0: 居中, 1: 刚才线在右侧需向右拐, 2: 刚才线在左侧需向左拐
volatile unsigned char last_dir = 0;


// =========================================================
// 基础延时与 TIM4 微秒定时器
// =========================================================
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


// =========================================================
// 超声波与 LED0 初始化 (Trig->PB10, Echo->PB11, LED0->PE5)
// =========================================================
void Sensor_Init(void) {
    RCC->APB2ENR |= (1 << 3) | (1 << 6); // 开启 GPIOB, GPIOE

    // PB8(蜂鸣器静音), PB10(Trig推挽输出 50MHz), PB11(Echo下拉输入)
    GPIOB->CRH &= 0xFFFF00F0;
    GPIOB->CRH |= 0x00008303;
    GPIOB->BRR  = (1 << 8);   // 静音
    GPIOB->BRR  = (1 << 10);  // Trig 初始置低
    GPIOB->BRR  = (1 << 11);  // Echo 下拉有效

    // PE5 (LED0) 推挽输出，初始熄灭 (高电平灭)
    GPIOE->CRL &= 0xFF0FFFFF;
    GPIOE->CRL |= 0x00300000;
    GPIOE->BSRR = (1 << 5);
}

float Get_Distance(void) {
    unsigned int time = 0;

    // 发射 20us 高电平脉冲
    GPIOB->BSRR = (1 << 10);
    Delay_us(20);
    GPIOB->BRR  = (1 << 10);

    // 等待 Echo 变高 (超时 20ms 返回 999)
    TIM4->CNT = 0;
    while ((GPIOB->IDR & (1 << 11)) == 0) {
        if (TIM4->CNT > 20000) return 999.0f;
    }

    // 记录高电平持续时长 (超时 25ms 返回 999)
    TIM4->CNT = 0;
    while ((GPIOB->IDR & (1 << 11)) != 0) {
        if (TIM4->CNT > 25000) return 999.0f;
    }

    time = TIM4->CNT;
    return (float)time / 58.0f; // 换算为厘米
}


// =========================================================
// 电机驱动与 TIM3 PWM 初始化
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
// 4路红外循迹模块配置与差速状态机 (带记忆与大弯优化)
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
        // ---------------- 1. 直行与微调 ----------------
        case 0b0110: // 中间两路完全居中
        case 0b1111: // 十字线/全黑
            Motor_SetSpeed(SPEED_BASE, SPEED_BASE);
            last_dir = 0;
            break;

        case 0b0010: // 稍微偏左，微幅向右修正
            Motor_SetSpeed(SPEED_BASE + 80, SPEED_BASE - 60);
            last_dir = 1;
            break;

        case 0b0100: // 稍微偏右，微幅向左修正
            Motor_SetSpeed(SPEED_BASE - 60, SPEED_BASE + 80);
            last_dir = 2;
            break;

        // ---------------- 2. 大弯 / 持续长弯 (强化差速) ----------------
        case 0b0001: // 最右探头踩线 -> 强力向右打舵 (拉大差速克服大弯冲出)
            Motor_SetSpeed(SPEED_HIGH, SPEED_LOW);
            last_dir = 1;
            break;

        case 0b1000: // 最左探头踩线 -> 强力向左打舵
            Motor_SetSpeed(SPEED_LOW, SPEED_HIGH);
            last_dir = 2;
            break;

        // ---------------- 3. 急弯 / 严重偏转 ----------------
        case 0b0011: // 右侧两路全黑
        case 0b0111:
            Motor_SetSpeed(SPEED_SPIN, -SPEED_SPIN / 4);
            last_dir = 1;
            break;

        case 0b1100: // 左侧两路全黑
        case 0b1110:
            Motor_SetSpeed(-SPEED_SPIN / 4, SPEED_SPIN);
            last_dir = 2;
            break;

        // ---------------- 4. 完全脱线自救 (依据最后出弯方向搜线) ----------------
        case 0b0000:
            if (last_dir == 1) {
                // 脱线前在右转 -> 维持右转搜线，防止沿切线直冲白地
                Motor_SetSpeed(SPEED_HIGH, 0);
            } else if (last_dir == 2) {
                // 脱线前在左转 -> 维持左转搜线
                Motor_SetSpeed(0, SPEED_HIGH);
            } else {
                // 纯直线丢线 -> 慢速前蹭探线
                Motor_SetSpeed(200, 200);
            }
            break;

        default:
            Motor_SetSpeed(SPEED_BASE, SPEED_BASE);
            break;
    }
}


// =========================================================
// 紧凑型避障绕行流程
// =========================================================
void Avoid_Obstacle(void) {
    unsigned int timeout = 0;
    unsigned char hit_line_cnt = 0;

    GPIOE->BRR = (1 << 5); // 点亮 LED0 警示

    // 【步骤 0】刹车缓冲，消除前冲惯性
    Motor_SetSpeed(0, 0);
    Delay_ms(150);

    // 【步骤 1】微幅快速右拐脱轨
    Motor_SetSpeed(AVOID_SPEED_FAST, AVOID_SPEED_SLOW);
    Delay_ms(TIME_TURN_RIGHT);

    // 【步骤 2】平稳超越障碍物 (速度 300)
    Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
    Delay_ms(TIME_FORWARD_PASS);

    // 【步骤 3】强化左打舵回切 (确保充足角度指向黑线)
    Motor_SetSpeed(AVOID_SPEED_LEFT_IN, AVOID_SPEED_FAST);
    Delay_ms(TIME_TURN_LEFT);

    // 【步骤 4】以 300 速度斜向抓线 (带双重防抖)
    Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
    while (timeout < 300) { // 最长找线 3.0 秒
        Tracking_Scan();

        if (track_l2 || track_l1 || track_r1 || track_r2) {
            hit_line_cnt++;
            if (hit_line_cnt >= 2) { // 连续 2 次采样确认踩黑线，过滤杂波
                break;
            }
        } else {
            hit_line_cnt = 0;
        }

        Delay_ms(10);
        timeout++;
    }

    // 【步骤 5】顺线微调：右打舵拉直车身与黑线平齐
    Motor_SetSpeed(480, 150);
    Delay_ms(TIME_PULL_STRAIGHT);

    // 避障回正后将偏航记忆归中，防止误判
    last_dir = 0;

    GPIOE->BSRR = (1 << 5); // 熄灭 LED0，切回常规循迹
}


// =========================================================
// 主程序
// =========================================================
int main(void) {
    unsigned char dist_sample_cnt = 0;

    Delay_ms(500);         // 延时等待供电稳定
    Timer4_Init();         // 初始化 TIM4 微秒定时器
    Sensor_Init();         // 初始化超声波与 LED0
    Motor_Init();          // 初始化电机 PWM
    Tracking_Init();       // 初始化 4路循迹引脚

    while (1) {
        // 分频采样超声波：每 30ms 测量一次，保证循迹主循环保持 10ms 快速响应
        if (++dist_sample_cnt >= 3) {
            dist_sample_cnt = 0;
            current_distance = Get_Distance();
        }

        // 遇障判断：有效距离 <= 12cm 进入紧凑绕障
        if (current_distance > 0.0f && current_distance <= STOP_THRESHOLD) {
            Avoid_Obstacle();
            current_distance = 100.0f; // 绕障完成后重置测距，防止重触发
        } else {
            // 前方畅通：以 300 基准速度稳定循线
            Tracking_Scan();
            Tracking_Drive();
        }

        Delay_ms(10); // 10ms 巡线控制周期
    }
}
