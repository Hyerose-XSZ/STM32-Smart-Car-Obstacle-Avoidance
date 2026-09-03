#include "stm32f10x.h"

// =========================================================
// 1. ??????????? (??320??,???????) —— ??????
// =========================================================
#define SPEED_BASE           450    // ????????
#define SPEED_CURVE_MIN       90    // ???????? (????????)
#define SPEED_DROP_RATE       50    // ?????? (???????????)

// PD ?? (?????????) —— ??????
#define KP   42.0f
#define KD   75.0f

static float last_error = 0.0f;     // ??? 0,??????????
static float current_error = 0.0f;
static uint8_t memory_dir = 1;      // ????: 1:?????, 2:?????

// =========================================================
// ?????? (???200??,??????????)
// =========================================================
#define STOP_THRESHOLD       12.0f  // ???????? (cm)

// ???????? 200 ??
#define AVOID_SPEED_RUN      300    // ???????? (200??)
#define AVOID_SPEED_FAST     460    // ?????????
#define AVOID_SPEED_SLOW      80    // ??1???? (???????????)
#define AVOID_SPEED_LEFT_IN   250    // ??3??????

// ???? 200 ??????
#define TIME_TURN_RIGHT      320    // ??1:??????????? (ms)
#define TIME_FORWARD_PASS    460    // ??2:?????????? (ms)
#define TIME_TURN_LEFT       430    // ??3:?????????? (ms)
#define TIME_PULL_STRAIGHT   180    // ??5:??????????? (ms)

volatile float current_distance = 100.0f;

// =========================================================
// 2. ??? TIM4 ???????? —— ??????
// =========================================================
void Delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 7200; j++);
}

void Timer4_Init(void) {
    RCC->APB1ENR |= (1 << 2); // ?? TIM4
    TIM4->PSC = 71;           // 1MHz (1us/Tick)
    TIM4->ARR = 0xFFFF;
    TIM4->CR1 |= (1 << 0);
}

void Delay_us(uint16_t us) {
    TIM4->CNT = 0;
    while (TIM4->CNT < us);
}

// =========================================================
// 3. LED0 ???? (PE5)???? (PB8) ? ????????
// =========================================================
void Sensor_Init(void) {
    RCC->APB2ENR |= (1 << 3) | (1 << 6); // GPIOB, GPIOE

    // PB8(?????), PB10(Trig????), PB11(Echo????)
    GPIOB->CRH &= 0xFFFF00F0;
    GPIOB->CRH |= 0x00008303;
    GPIOB->BRR  = (1 << 8);
    GPIOB->BRR  = (1 << 10);
    GPIOB->BRR  = (1 << 11);

    // PE5 (LED0) ????,????
    GPIOE->CRL &= 0xFF0FFFFF;
    GPIOE->CRL |= 0x00300000;
    GPIOE->BSRR = (1 << 5);
}

// ??????? (PB10 Trig, PB11 Echo)
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
// 4. ????? TIM3 ?? 1kHz PWM —— ??????
// =========================================================
void Motor_Init(void) {
    RCC->APB2ENR |= (1 << 2); // GPIOA
    RCC->APB1ENR |= (1 << 1); // TIM3

    // PA0, PA1, PA4, PA5 ????; PA6, PA7 ??????
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
    // ???? (IN1: PA0, IN2: PA1, ENA: PA6)
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

    // ???? (IN3: PA4, IN4: PA5, ENB: PA7)
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
// 5. 4??????????????? —— ??????
// =========================================================
void Tracking_Init(void) {
    RCC->APB2ENR |= (1 << 4); // ?? GPIOC

    // PC0~PC3 ????
    GPIOC->CRL &= 0xFFFF0000;
    GPIOC->CRL |= 0x00008888;
    GPIOC->BSRR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
}

// ???? —— ??????
float Tracking_GetError(void) {
    uint8_t l2 = (GPIOC->IDR & (1 << 0)) ? 1 : 0; // IN4 (??)
    uint8_t l1 = (GPIOC->IDR & (1 << 1)) ? 1 : 0; // IN3 (??)
    uint8_t r1 = (GPIOC->IDR & (1 << 2)) ? 1 : 0; // IN2 (??)
    uint8_t r2 = (GPIOC->IDR & (1 << 3)) ? 1 : 0; // IN1 (??)

    uint8_t state = (l2 << 3) | (l1 << 2) | (r1 << 1) | r2;

    switch (state) {
        case 0b0110: return 0.0f; // ??

        // ???? (????)
        case 0b0010: memory_dir = 2; return  1.0f; // ???
        case 0b0011: memory_dir = 2; return  2.5f; // ????
        case 0b0001:
        case 0b0111: memory_dir = 2; return  4.2f; // ??????

        // ???? (????)
        case 0b0100: memory_dir = 1; return -1.0f; // ???
        case 0b1100: memory_dir = 1; return -2.5f; // ????
        case 0b1000:
        case 0b1110: memory_dir = 1; return -4.2f; // ??????

        case 0b1111: return 0.0f;

        // ????
        case 0b0000: return 99.0f;

        default: return 0.0f;
    }
}

// =========================================================
// 6. ????:???? + ??????/???? —— ??????
// =========================================================
void PID_Tracking_Loop(void) {
    float raw_error = Tracking_GetError();
    float pid_output = 0.0f;
    float derivative = 0.0f;
    float abs_error = 0.0f;
    int current_base = 0;
    int left_motor = 0;
    int right_motor = 0;

    // 1. ??????:??????????????
    if (raw_error == 99.0f) {
        GPIOE->BRR = (1 << 5); // ?? LED0
        if (memory_dir == 2) {
            Motor_SetSpeed(350, -260); // ???????
        } else {
            Motor_SetSpeed(-260, 350); // ???????
        }
        return;
    }

    GPIOE->BSRR = (1 << 5); // ?? LED0
    current_error = raw_error;

    // 2. ???????:???????,???????? 200,??????? 90
    abs_error = (current_error >= 0.0f) ? current_error : -current_error;
    current_base = SPEED_BASE - (int)(abs_error * SPEED_DROP_RATE);
    if (current_base < SPEED_CURVE_MIN) {
        current_base = SPEED_CURVE_MIN;
    }

    // 3. PD ????
    derivative = current_error - last_error;
    pid_output = (KP * current_error) + (KD * derivative);
    last_error = current_error;

    // 4. ????????????
    // -------------------------------------------------------------
    // ??? A:??/????? (|Error| >= 4.0)?
    // ????? 380 ??,????? -180 ????,??????
    if (abs_error >= 4.0f) {
        if (current_error > 0.0f) {
            left_motor  = 380;
            right_motor = -180; // ???
        } else {
            left_motor  = -180; // ???
            right_motor = 380;
        }
    }
    // ??? B:???? (|Error| >= 2.0)?
    // ????,?????? -90 ??????,????????
    else if (abs_error >= 2.0f) {
        if (current_error > 0.0f) {
            left_motor  = current_base + 120;
            right_motor = -90;
        } else {
            left_motor  = -90;
            right_motor = current_base + 120;
        }
    }
    // ??? C:????? (|Error| < 2.0)?
    // ??????,???????,????,????????
    else {
        left_motor  = current_base + (int)pid_output;
        right_motor = current_base - (int)pid_output;
    }

    // 5. ??????
    Motor_SetSpeed(left_motor, right_motor);
}

// =========================================================
// 7. ??????? (200 ???? + ????????)
// =========================================================
void Avoid_Obstacle(void) {
    unsigned int timeout = 0;
    unsigned char hit_line_cnt = 0;

    GPIOE->BRR = (1 << 5); // ?? LED0 ?????

    // ?? 0:????????
    Motor_SetSpeed(0, 0);
    Delay_ms(150);

		Motor_SetSpeed(-450, -450);
		Delay_ms(150);
		
		Motor_SetSpeed(0, 0);
		Delay_ms(100);
		
    // ?? 1:????????? (??????????)
    Motor_SetSpeed(AVOID_SPEED_FAST, AVOID_SPEED_SLOW);
    Delay_ms(TIME_TURN_RIGHT);

    // ?? 2:? 200 ?????????
    Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
    Delay_ms(590);

    // ?? 3:????????
		Motor_SetSpeed(-300, 200);
		Delay_ms(20);
    Motor_SetSpeed(0, 460);
    Delay_ms(TIME_TURN_LEFT);

    // ?? 4:? 200 ?????????? (???? 3 ?)
    Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
    while (timeout < 300) {
        uint8_t l2 = (GPIOC->IDR & (1 << 0)) ? 1 : 0;
        uint8_t l1 = (GPIOC->IDR & (1 << 1)) ? 1 : 0;
        uint8_t r1 = (GPIOC->IDR & (1 << 2)) ? 1 : 0;
        uint8_t r2 = (GPIOC->IDR & (1 << 3)) ? 1 : 0;

        // ?????? 2 ??????????
        if (l2 || l1 || r1 || r2) {
            if (++hit_line_cnt >= 2) break;
        } else {
            hit_line_cnt = 0;
        }

        Delay_ms(10);
        timeout++;
    }

    // ?? 5:??????????????
    Motor_SetSpeed(260, 80);
    Delay_ms(TIME_PULL_STRAIGHT);

    // ?? 6:?????????? PID
    last_error = 0.0f;
    current_error = 0.0f;
    memory_dir = 2; // ??????????,????
    GPIOE->BSRR = (1 << 5); // ?? LED0
}

// =========================================================
// 8. ???
// =========================================================
int main(void) {
    uint8_t dist_cycle_cnt = 0;

    Sensor_Init();
    Timer4_Init();
    Motor_Init();
    Tracking_Init();

    // ????:LED0 ???? 2 ?????? —— ??????
    GPIOE->BRR = (1 << 5);  Delay_ms(100);
    GPIOE->BSRR = (1 << 5); Delay_ms(100);
    GPIOE->BRR = (1 << 5);  Delay_ms(100);
    GPIOE->BSRR = (1 << 5); Delay_ms(200);

    while (1) {
        // ???????:? 30~40ms ????,???? 2ms ????????
        if (++dist_cycle_cnt >= 10) {
            dist_cycle_cnt = 0;
            current_distance = Get_Distance();
        }

        // ?????:???? 12cm ????,??????????
        if (current_distance > 0.0f && current_distance <= STOP_THRESHOLD) {
            Avoid_Obstacle();
            current_distance = 100.0f; // ???????????
        } else {
            PID_Tracking_Loop();
        }

        Delay_ms(2); // ?? 2ms ?????? —— ??????
    }
}