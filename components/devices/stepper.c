#include "stepper.h"
#include "main.h"
#include "tim.h"

#define STEPPER_MMPS2HZ (20)
// TIM1 CH1 as PFM output
// PB14 as DIR output

void set_freq(int freq)
{
    // MCLK = 168M, PSC=167
    // Ftick = MCLK / (PSC+1) = 1MHz
    // PWM Period = 1 / Ftick * (Counter+1)
    // PWM Frequency = Ftick / (Counter + 1)
    // Duty = CCR / (Counter + 1)

    // Counter = 10000 / f - 1
    // CCR = (Counter + 1) / 2

    if (freq == 0)
    {
        // disable output
        TIM1->CCR1 = 0;
    }
    else 
    {
        TIM1->ARR = 1000000 / freq - 1; // counter period
        TIM1->CCR1 = (TIM1->ARR + 1) / 2;
    }
}

void set_dir(int dir)
{
    if (dir >= 0)
    {
        HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, GPIO_PIN_SET);
    }
    else 
    {
        HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, GPIO_PIN_RESET);
    }
}

void set_stepper_speed(int speed) // unit: mm/s
{
    set_dir(speed);
    speed = speed < 0 ? -speed : speed;
    set_freq(speed * STEPPER_MMPS2HZ);
}