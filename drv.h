/**
  ******************************************************************************
  * @file           : drv.h
  * @brief          : Header for drv.c file.
  ******************************************************************************
  */
#ifndef __DRV_H
#define __DRV_H

#include "tim.h"
#include "i2c.h"
#include "adc.h"

/**
  ******************************************************************************
  * @brief   DWT microseconds delay functions
  ******************************************************************************
  */
#define N_TICK_CALIBR  50 // for CLKfreq 84 MHz
typedef enum {DWT_TICK_LESS = 0, DWT_TICK_MORE = 1} DWTTickState;
extern uint32_t  DWT_Scale;
void InitDelayUs ();
void SetTimeDelayUs ();
DWTTickState TimeDelayUs (uint32_t microseconds);
void DelayUs (uint32_t microseconds);

/**
  ******************************************************************************
  * @brief   SysTick functions
  ******************************************************************************
  */
typedef enum {COUNT_TICK_LESS = 0, COUNT_TICK_MORE = 1} CountTickState;
typedef struct{
	uint32_t  time;     ///< The timeout setting
	uint32_t  count;    ///< Counter of the current time
}TypeSysTickDelay;

CountTickState DeltaSysTick (uint32_t cnt, uint32_t tdat);
void InitSysTickDelay (TypeSysTickDelay *delay, uint32_t time);
CountTickState SysTickDelay (TypeSysTickDelay *delay);

/**
  ******************************************************************************
  * @brief   Pulser functions
  ******************************************************************************
  */
typedef enum {PULSE_ON, PULSE_OFF, PULSE_ONE, PULSE_REPEAT} PulseState;
typedef enum {_PAUSE, _PULSE} PulseStt;

typedef struct{
	PulseState     state;
	uint32_t       tpulse;   // msec
	uint32_t       tpause;   // msec
    uint32_t       cnt;
    PulseStt       stt;
    void           *hdr;
    void (*_pulse_ON)  (void *hdr);
    void (*_pulse_OFF) (void *hdr);
}TypePulser;

void InitPulser (TypePulser *pls);
void SetPulser (TypePulser *pls);
void Pulser_Handler (TypePulser *pls);

/**
  ******************************************************************************
  * @brief   Leds functions
  ******************************************************************************
  */
// LED_HIGH: port - high  -> Led is ON (emits)
// LED_LOW:  port - low   -> Led is ON (emits)
typedef enum {LED_HIGH, LED_LOW} LedPolarity;

typedef struct{
	GPIO_TypeDef  *port;     // led_pin port
	uint16_t       pin;      // led_pin
	LedPolarity    polarity;
	TypePulser     pls;
}TypeLed;

void InitLed (TypeLed *led, GPIO_TypeDef* port, uint16_t pin, LedPolarity pol);
void SetLed (TypeLed *led, PulseState pstate, uint32_t tpulse, uint32_t tpause);
void Led_Handler (TypeLed *led);

/**
  ******************************************************************************
  * @brief   Beeper functions
  ******************************************************************************
  */
#define PERIOD_BEEPER_TON    20930
typedef struct{
	TIM_HandleTypeDef *htim;
	TypePulser     pls;
}TypeBeeper;

void InitBeeper  (TypeBeeper *beep, TIM_HandleTypeDef *htim);
void SetBeeper (TypeBeeper *beep, PulseState pstate, uint32_t tpulse, uint32_t tpause);
void Beeper_Handler (TypeBeeper *beep);

/**
  ******************************************************************************
  * @brief  PWM Leds functions
  ******************************************************************************
  */
typedef enum {_PWM_UP, _PWM_DOWN,  _PWM_STOP} PWMStt;
typedef struct{
	TypeLed    *led;
	TIM_HandleTypeDef *htim;
	uint32_t    tim_chan;
	uint32_t    tim_cc;
	uint16_t    period;
	uint16_t    delta;
	uint16_t    pwm_cnt;
	PWMStt      stt;
}TypePWMLed;

void InitPWMLed (TypePWMLed *pwm_led, TypeLed *led, TIM_HandleTypeDef *htim, uint32_t tim_chan, uint32_t tim_cc);
void StartPWMLedTimer (TypePWMLed *pwm_led);
void StartPWMLed (TypePWMLed *pwm_led, uint16_t period);
void StopPWMLed (TypePWMLed *pwm_led);
void PWMLed_Handler (TypePWMLed *pwm_led);

/**
  ******************************************************************************
  * @brief   Button functions
  ******************************************************************************
  */
#define TIME_BUTTON_DELAY   50 // msec
#define TIME_BUTTON_BEEP    50 // msec
// BUTTON_HIGH: button is ON (press) -> port is high
// BUTTON_LOW:  button is ON (press) -> port is low
typedef enum {BUTTON_HIGH, BUTTON_LOW} ButtonPolarity;
typedef enum {BUTTON_STATE_OFF, BUTTON_STATE_ON}  ButtonState;
typedef enum {BUTTON_FLAG_RESET, BUTTON_FLAG_SET} ButtonFlag;
typedef enum {BUTTON_BEEP_OFF, BUTTON_BEEP_ON} ButtonBeep;
typedef struct{
	GPIO_TypeDef  *port;     // button_pin port
	uint16_t       pin;      // button_pin
	ButtonPolarity pol;
	ButtonState    state;
	ButtonState    _state;
	ButtonFlag     _flag_change;
	ButtonFlag     flag_toON;
	ButtonFlag     flag_toOFF;
	uint32_t       cnt_delay;
	TypeBeeper     *beep;
	ButtonBeep     beep_toON;
	ButtonBeep     beep_toOFF;
}TypeButton;

void InitButton (TypeButton *btn, GPIO_TypeDef* port, uint16_t pin, ButtonPolarity pol, TypeBeeper *beep);
void SetButtonBeep (ButtonBeep *beep_to, ButtonBeep beep_stt);
ButtonState GetButtonState (TypeButton *btn);
ButtonFlag  GetButtonFlag  (ButtonFlag *btn_flag);
void ResetButtonFlag (ButtonFlag *btn_flag);
void Button_Handler (TypeButton *btn);


/**
  ******************************************************************************
  * @brief   EEPROM functions
  ******************************************************************************
  */
typedef struct{
	I2C_HandleTypeDef *hi2c;
	uint8_t dev_adr;
}TypeEEPROM;

void InitEEPROM (TypeEEPROM *eeprom, I2C_HandleTypeDef *hi2c, uint8_t dev_adr);
HAL_StatusTypeDef WriteEEPROM (TypeEEPROM *eeprom, uint16_t adr, uint16_t dat);
uint16_t ReadEEPROM (TypeEEPROM *eeprom, uint16_t adr);

/**
  ******************************************************************************
  * @brief   Recursive filter functions
  ******************************************************************************
  */
typedef struct{
	uint8_t   Kf;
	uint32_t  buffer;
}TypeRecFilter;
void InitRecFilter   (TypeRecFilter *rfilter, uint16_t dat, uint8_t Kf);
uint16_t GetRecFiltr (TypeRecFilter *rfilter, uint16_t dat);

/**
  ******************************************************************************
  * @brief   ADC functions
  ******************************************************************************
  */
typedef enum {POLARITY_NORMAL, POLARITY_REVERSE} ADC_Polarity;
typedef struct{
	ADC_HandleTypeDef *hadc;
	TypeRecFilter   ADCfilter;
	ADC_Polarity    pol;
}TypeADC;
void InitADC (TypeADC *adc, ADC_HandleTypeDef *hadc, uint8_t Kf, ADC_Polarity pol);
uint16_t GetADC (TypeADC *adc);




#endif /* __DRV_H */
