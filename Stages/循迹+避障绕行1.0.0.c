#include "stm32f10x.h"

// =========================================================
// 1. ???????? (?????? 300)
// =========================================================
#define SPEED_BASE          300    // ???????? (0~1000,?? 300)
#define SPEED_HIGH          480    // ?????????
#define SPEED_LOW           150    // ????????? (???????)
#define SPEED_SPIN          450    // ??????????

// =========================================================
// 2. ???????? (???????????????)
// =========================================================
#define STOP_THRESHOLD      12.0f  // ?????? (cm)

// ????
#define AVOID_SPEED_RUN     300    // ??????????? (?? 300)
#define AVOID_SPEED_FAST    460    // ????????
#define AVOID_SPEED_SLOW    130    // ???????
#define AVOID_SPEED_LEFT_IN  60    // ???????

// ???? (??: ms)
#define TIME_TURN_RIGHT     150    // ??1:????????
#define TIME_FORWARD_PASS   480    // ??2:???????
#define TIME_TURN_LEFT      380    // ??3:?????? (?????????)
#define TIME_PULL_STRAIGHT  120    // ??5:????????

// ????
volatile float current_distance = 100.0f; // ????? (cm)
volatile unsigned char track_l2 = 0;      // ?? PC0 (? IN4)
volatile unsigned char track_l1 = 0;      // ?? PC1 (? IN3)
volatile unsigned char track_r1 = 0;      // ?? PC2 (? IN2)
volatile unsigned char track_r2 = 0;      // ?? PC3 (? IN1)


// =========================================================
// ????? TIM4 ?????
// =========================================================
void Delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 7200; j++);
}

void Timer4_Init(void) {
    RCC->APB1ENR |= (1 << 2); // ?? TIM4 ??
    TIM4->PSC = 71;           // 72MHz / 72 = 1MHz (1us/??)
    TIM4->ARR = 0xFFFF;
    TIM4->CR1 |= (1 << 0);
}

void Delay_us(unsigned int us) {
    TIM4->CNT = 0;
    while (TIM4->CNT < us);
}


// =========================================================
// ???? LED0 ??? (Trig->PB10, Echo->PB11, LED0->PE5)
// =========================================================
void Sensor_Init(void) {
    RCC->APB2ENR |= (1 << 3) | (1 << 6); // ?? GPIOB, GPIOE

    // PB8(?????), PB10(Trig???? 50MHz), PB11(Echo????)
    GPIOB->CRH &= 0xFFFF00F0;
    GPIOB->CRH |= 0x00008303;
    GPIOB->BRR  = (1 << 8);   // ??
    GPIOB->BRR  = (1 << 10);  // Trig ????
    GPIOB->BRR  = (1 << 11);  // Echo ????

    // PE5 (LED0) ????,???? (????)
    GPIOE->CRL &= 0xFF0FFFFF;
    GPIOE->CRL |= 0x00300000;
    GPIOE->BSRR = (1 << 5);
}

float Get_Distance(void) {
    unsigned int time = 0;

    // ?? 20us ?????
    GPIOB->BSRR = (1 << 10);
    Delay_us(20);
    GPIOB->BRR  = (1 << 10);

    // ?? Echo ?? (?? 20ms ?? 999)
    TIM4->CNT = 0;
    while ((GPIOB->IDR & (1 << 11)) == 0) {
        if (TIM4->CNT > 20000) return 999.0f;
    }

    // ????????? (?? 25ms ?? 999)
    TIM4->CNT = 0;
    while ((GPIOB->IDR & (1 << 11)) != 0) {
        if (TIM4->CNT > 25000) return 999.0f;
    }

    time = TIM4->CNT;
    return (float)time / 58.0f; // ?????
}


// =========================================================
// ????? TIM3 PWM ???
// =========================================================
void Motor_Init(void) {
    RCC->APB2ENR |= (1 << 2); // ?? GPIOA
    RCC->APB1ENR |= (1 << 1); // ?? TIM3

    // PA0,1,4,5 ??????, PA6,7 ?????? (TIM3_CH1/CH2 PWM)
    GPIOA->CRL &= 0x0000FF00;
    GPIOA->CRL |= 0xBB330033;
    GPIOA->BRR = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5);

    TIM3->PSC = 71;           // ??? 72 -> 1MHz
    TIM3->ARR = 1000 - 1;     // 1kHz PWM
    TIM3->CCMR1 |= (0x6 << 4) | (1 << 3) | (0x6 << 12) | (1 << 11);
    TIM3->CCER  |= (1 << 0) | (1 << 4);
    TIM3->CR1   |= (1 << 0);
}

void Motor_SetSpeed(int speedLeft, int speedRight) {
    // ????
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

    // ????
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
// 4??????????????? (300??????)
// =========================================================
void Tracking_Init(void) {
    RCC->APB2ENR |= (1 << 4); // ?? GPIOC

    // PC0~PC3 ??????? (MODE=00, CNF=10 -> 0x8)
    GPIOC->CRL &= 0xFFFF0000;
    GPIOC->CRL |= 0x00008888;
    GPIOC->BSRR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
}

void Tracking_Scan(void) {
    // ?????????,??????,??????? (1)
    track_l2 = ((GPIOC->IDR & (1 << 0)) != 0) ? 1 : 0; // IN4 (??)
    track_l1 = ((GPIOC->IDR & (1 << 1)) != 0) ? 1 : 0; // IN3 (??)
    track_r1 = ((GPIOC->IDR & (1 << 2)) != 0) ? 1 : 0; // IN2 (??)
    track_r2 = ((GPIOC->IDR & (1 << 3)) != 0) ? 1 : 0; // IN1 (??)
}

void Tracking_Drive(void) {
    unsigned char sensor_state = (track_l2 << 3) | (track_l1 << 2) | (track_r1 << 1) | track_r2;

    switch (sensor_state) {
        // 1. ??:??????,???????,???????
        case 0b0110:
        case 0b0100:
        case 0b0010:
        case 0b1111:
            Motor_SetSpeed(SPEED_BASE, SPEED_BASE);
            break;

        // 2. ??????:????????
        case 0b0001: // ?????? -> ????
            Motor_SetSpeed(SPEED_HIGH, SPEED_LOW);
            break;
        case 0b1000: // ?????? -> ????
            Motor_SetSpeed(SPEED_LOW, SPEED_HIGH);
            break;

        // 3. ?? / ????:??????????
        case 0b0011: // ?????? -> ???????
        case 0b0111:
            Motor_SetSpeed(SPEED_SPIN, -SPEED_SPIN / 3);
            break;
        case 0b1100: // ?????? -> ???????
        case 0b1110:
            Motor_SetSpeed(-SPEED_SPIN / 3, SPEED_SPIN);
            break;

        // 4. ???? (0b0000):?????????
        case 0b0000:
            Motor_SetSpeed(SPEED_BASE - 60, SPEED_BASE - 60);
            break;

        default:
            Motor_SetSpeed(SPEED_BASE, SPEED_BASE);
            break;
    }
}


// =========================================================
// ??????????? (?????????)
// =========================================================
void Avoid_Obstacle(void) {
    unsigned int timeout = 0;
    unsigned char hit_line_cnt = 0;

    GPIOE->BRR = (1 << 5); // ?? LED0 ??

    // ??? 0?????,??????
    Motor_SetSpeed(0, 0);
    Delay_ms(150);

    // ??? 1?????????
    Motor_SetSpeed(AVOID_SPEED_FAST, AVOID_SPEED_SLOW);
    Delay_ms(TIME_TURN_RIGHT);

    // ??? 2???????? (?? 300)
    Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
    Delay_ms(TIME_FORWARD_PASS);

    // ??? 3???????? (???????????)
    Motor_SetSpeed(AVOID_SPEED_LEFT_IN, AVOID_SPEED_FAST);
    Delay_ms(TIME_TURN_LEFT);

    // ??? 4?? 300 ?????? (?????)
    Motor_SetSpeed(AVOID_SPEED_RUN, AVOID_SPEED_RUN);
    while (timeout < 300) { // ???? 3.0 ?
        Tracking_Scan();

        if (track_l2 || track_l1 || track_r1 || track_r2) {
            hit_line_cnt++;
            if (hit_line_cnt >= 2) { // ?? 2 ????????,????
                break;
            }
        } else {
            hit_line_cnt = 0;
        }

        Delay_ms(10);
        timeout++;
    }

    // ??? 5?????:????????????
    Motor_SetSpeed(480, 150);
    Delay_ms(TIME_PULL_STRAIGHT);

    GPIOE->BSRR = (1 << 5); // ?? LED0,??????????
}


// =========================================================
// ???
// =========================================================
int main(void) {
    unsigned char dist_sample_cnt = 0;

    Delay_ms(500);         // ????????
    Timer4_Init();         // ??? TIM4 ?????
    Sensor_Init();         // ??????? LED0
    Motor_Init();          // ????? PWM
    Tracking_Init();       // ??? 4?????

    while (1) {
        // ???????:? 30ms ????,????????? 10ms ????
        if (++dist_sample_cnt >= 3) {
            dist_sample_cnt = 0;
            current_distance = Get_Distance();
        }

        // ????:???? <= 12cm ??????
        if (current_distance > 0.0f && current_distance <= STOP_THRESHOLD) {
            Avoid_Obstacle();
            current_distance = 100.0f; // ?????????,?????
        } else {
            // ????:? 300 ????????
            Tracking_Scan();
            Tracking_Drive();
        }

        Delay_ms(10); // 10ms ??????
    }
}
