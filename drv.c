/**
  ******************************************************************************
  * @file           : drv.c
  * @brief          : DRV program body
  ******************************************************************************
  */
#include "drv.h"

/**
  ******************************************************************************
  * @brief   DWT microseconds delay functions
  ******************************************************************************
  */
uint32_t  DWT_Scale;

void InitDelayUs (){
	DWT_Scale = HAL_RCC_GetHCLKFreq() / 1000000;
	CoreDebug -> DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT -> CYCCNT = 0;
	DWT -> CTRL |= 1;
}

void SetTimeDelayUs (){
	DWT -> CYCCNT = 0;
}

DWTTickState TimeDelayUs (uint32_t microseconds){
	uint32_t ticks = microseconds * DWT_Scale - N_TICK_CALIBR;;
	if (DWT -> CYCCNT < ticks) { return (DWT_TICK_LESS);}
	return (DWT_TICK_MORE);
}

void DelayUs(uint32_t microseconds){
	uint32_t ticks = microseconds * DWT_Scale - N_TICK_CALIBR;
	DWT -> CYCCNT = 0;
	while (DWT -> CYCCNT < ticks);
}


/**
  ******************************************************************************
  * @brief   SysTick functions
  ******************************************************************************
  */
CountTickState DeltaSysTick (uint32_t tcnt, uint32_t tdat){
	uint32_t delta = HAL_GetTick() - tcnt;
	if (delta >= tdat) {
		return (COUNT_TICK_MORE);
	}
	return (COUNT_TICK_LESS);
}

void InitSysTickDelay (TypeSysTickDelay *delay, uint32_t time){
	delay -> count = HAL_GetTick();
	delay -> time = time;
}

CountTickState SysTickDelay (TypeSysTickDelay *delay){
	if (DeltaSysTick (delay -> count, delay -> time)){
		delay -> count = HAL_GetTick();
		return (COUNT_TICK_MORE);
	}
	return (COUNT_TICK_LESS);
}



/**
  ******************************************************************************
  * @brief   Pulser functions
  ******************************************************************************
  */
void InitPulser (TypePulser *pls){
	pls -> state = PULSE_OFF;
}
void SetPulser (TypePulser *pls){
	void (*pf)(void *hdr);
	switch (pls -> state){
	case PULSE_ON : 	pf = pls ->_pulse_ON;
						pf (pls -> hdr);
						break;
	case PULSE_OFF :	pf = pls ->_pulse_OFF;
						pf (pls -> hdr);
						break;
	case PULSE_ONE:		pls -> cnt = HAL_GetTick();
						pf = pls ->_pulse_ON;
						pf (pls -> hdr);
						break;
	case PULSE_REPEAT:	pls -> stt = _PULSE;
						pls -> cnt = HAL_GetTick();
						pf = pls ->_pulse_ON;
						pf (pls -> hdr);
						break;
	}
}

void Pulser_Handler (TypePulser *pls){
	void (*pf)(void *hdr);
	switch (pls -> state){
	case PULSE_ON :  	break;
	case PULSE_OFF: 	break;
	case PULSE_ONE: 	if(DeltaSysTick (pls -> cnt, pls -> tpulse)){
							pls -> state = PULSE_OFF;
							pf = pls ->_pulse_OFF;
							pf (pls -> hdr);
					 	}
						break;
	case PULSE_REPEAT: 	if (pls -> stt ==  _PULSE){
							if (DeltaSysTick (pls -> cnt, pls -> tpulse)){
								pls -> cnt = HAL_GetTick();
								pls -> stt = _PAUSE;
								pf = pls ->_pulse_OFF;
								pf (pls -> hdr);
							}
						}
						else{
							if (DeltaSysTick (pls -> cnt, pls -> tpause)){
								pls -> cnt = HAL_GetTick();
								pls -> stt = _PULSE;
								pf = pls ->_pulse_ON;
								pf (pls -> hdr);
							}
						}
						break;
	}
}

/**
  ******************************************************************************
  * @brief   Leds functions
  ******************************************************************************
  */
static void _led_on (TypeLed *led){
	if ((led ->polarity) == LED_HIGH){
		HAL_GPIO_WritePin (led -> port, led -> pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin (led -> port, led -> pin, GPIO_PIN_RESET);
	}
}

static void _led_off (TypeLed *led){
	if ((led ->polarity) == LED_HIGH){
		HAL_GPIO_WritePin (led -> port, led -> pin, GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin (led -> port, led -> pin, GPIO_PIN_SET);
	}
}

void InitLed (TypeLed *led, GPIO_TypeDef* port, uint16_t pin, LedPolarity pol){
	led -> port = port;
	led -> pin = pin;
	led -> polarity = pol;
	led -> pls.hdr = (void *) led;
	led -> pls._pulse_ON = (void *) _led_on;
	led -> pls._pulse_OFF = (void *) _led_off;
	InitPulser (&(led -> pls));
	_led_off (led);

}

void SetLed (TypeLed *led, PulseState pstate, uint32_t tpulse, uint32_t tpause){
	led -> pls.state = pstate;
	led -> pls.tpulse = tpulse;
	led -> pls.tpause = tpause;
	SetPulser (&(led -> pls));
}

void Led_Handler (TypeLed *led){
	Pulser_Handler (&(led -> pls));
}

/**
  ******************************************************************************
  * @brief   Beeper functions
  ******************************************************************************
  */
static void _beep_on (TypeBeeper *beep){
	HAL_TIM_PWM_Start (beep -> htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start (beep -> htim, TIM_CHANNEL_2);
}
static void _beep_off (TypeBeeper *beep){
	HAL_TIM_PWM_Stop (beep -> htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop (beep -> htim, TIM_CHANNEL_2);
}

void InitBeeper  (TypeBeeper *beep, TIM_HandleTypeDef *htim){
	beep -> htim = htim;
	beep -> pls.hdr = (void *) beep;
	beep -> pls._pulse_ON = (void *) _beep_on;
	beep -> pls._pulse_OFF = (void *) _beep_off;
	InitPulser (&(beep -> pls));
	_beep_off (beep);
	__HAL_TIM_SET_AUTORELOAD(beep -> htim, PERIOD_BEEPER_TON);
	__HAL_TIM_SET_COMPARE(beep -> htim, TIM_CHANNEL_1, PERIOD_BEEPER_TON / 2);
	__HAL_TIM_SET_COMPARE(beep -> htim, TIM_CHANNEL_2, PERIOD_BEEPER_TON / 2);
}

void SetBeeper (TypeBeeper *beep, PulseState pstate, uint32_t tpulse, uint32_t tpause){
	beep -> pls.state = pstate;
	beep -> pls.tpulse = tpulse;
	beep -> pls.tpause = tpause;
	SetPulser (&(beep -> pls));
}

void Beeper_Handler (TypeBeeper *beep){
	Pulser_Handler (&(beep -> pls));
}

/**
  ******************************************************************************
  * @brief  PWM Leds functions
  ******************************************************************************
  */
/* SET CUBE MX timer: prescaler - 720 (for SystemFreq 72MHz), period (autoreload register) = 1000
 * Input frequency of the PWM timer - 10 000 Hz
 * the prescaler of the timer will be equal to SystemCoreClock / 10000 (for 72MHz - 720)
 * the frequency of the PWM - 100 Hz
 * the autoreload register is equal to 1000
 * delta PWM: 1000 / (period / 10)
 * during the period of PWM, it grows and falls (period/2 - grow, period/2 -falls)
 * so delta PWM = 20000/period
 */

void InitPWMLed (TypePWMLed *pwm_led, TypeLed *led, TIM_HandleTypeDef *htim, uint32_t tim_chan, uint32_t tim_cc){
	pwm_led ->led = led;
	pwm_led -> htim = htim;
	pwm_led -> tim_chan = tim_chan;
	pwm_led -> tim_cc = tim_cc;
	pwm_led -> stt = _PWM_STOP;
	HAL_TIM_Base_Stop_IT(pwm_led -> htim);
	HAL_TIM_PWM_Stop_IT (pwm_led -> htim, pwm_led -> tim_chan);
}
void StartPWMLedTimer (TypePWMLed *pwm_led){
	HAL_TIM_Base_Start_IT(pwm_led -> htim);
}

void StartPWMLed (TypePWMLed *pwm_led, uint16_t period){
	pwm_led -> led -> pls.state = PULSE_OFF;
	_led_off (pwm_led -> led);
	pwm_led -> period = period;
	pwm_led -> delta = 20000  / period;
	pwm_led -> pwm_cnt = 1;
	pwm_led -> stt = _PWM_UP;
	__HAL_TIM_SET_COMPARE(pwm_led -> htim, pwm_led -> tim_chan, 1);
	HAL_TIM_Base_Start_IT(pwm_led -> htim);
	HAL_TIM_PWM_Start_IT (pwm_led -> htim, pwm_led -> tim_chan);
}

void StopPWMLed (TypePWMLed *pwm_led){
	pwm_led -> stt = _PWM_STOP;
	HAL_TIM_PWM_Stop_IT (pwm_led -> htim, pwm_led -> tim_chan);
	pwm_led -> led -> pls.state = PULSE_OFF;
	_led_off (pwm_led -> led);
}

void PWMLed_Handler (TypePWMLed *pwm_led){
	if (pwm_led -> stt == _PWM_STOP){
		return;
	}
	if((__HAL_TIM_GET_FLAG (pwm_led -> htim, TIM_FLAG_UPDATE))){
			_led_on (pwm_led -> led);
		}
	if(__HAL_TIM_GET_FLAG (pwm_led -> htim, pwm_led -> tim_cc)){
			_led_off (pwm_led -> led);
	}
	if ((pwm_led -> stt == _PWM_UP) && (pwm_led -> pwm_cnt += pwm_led -> delta) >= 1000){
			pwm_led -> stt = _PWM_DOWN;
			pwm_led -> pwm_cnt = 999;
	}
	if ((pwm_led -> stt == _PWM_DOWN) && (pwm_led -> pwm_cnt -= pwm_led -> delta) < pwm_led -> delta){
				pwm_led -> stt = _PWM_UP;
				pwm_led -> pwm_cnt = 1;
		}
	__HAL_TIM_SET_COMPARE(pwm_led -> htim, pwm_led -> tim_chan, pwm_led -> pwm_cnt);
}

/**
  ******************************************************************************
  * @brief   Button functions
  ******************************************************************************
  */
static ButtonState _state_button (TypeButton *btn){
	if (btn -> pol == BUTTON_HIGH){
		if (HAL_GPIO_ReadPin(btn -> port, btn -> pin)){ return (BUTTON_STATE_ON); }
		else { return (BUTTON_STATE_OFF);}
	}
	else{
		if (HAL_GPIO_ReadPin(btn -> port, btn -> pin)){ return (BUTTON_STATE_OFF); }
		else { return (BUTTON_STATE_ON);}
	}
}
static void _set_flag_button (TypeButton *btn, ButtonFlag *flag_btn, ButtonBeep *beep_btn ){
	*flag_btn =  BUTTON_FLAG_SET;
	if ((*beep_btn) == BUTTON_BEEP_ON){
		SetBeeper (btn -> beep, PULSE_ONE, TIME_BUTTON_BEEP , 0);
	}
}

void InitButton (TypeButton *btn, GPIO_TypeDef* port, uint16_t pin, ButtonPolarity pol, TypeBeeper *beep){
	btn -> port = port;
	btn -> pin = pin;
	btn -> pol = pol;
	btn -> beep = beep;
	btn -> beep_toON  = BUTTON_BEEP_OFF;
	btn -> beep_toOFF = BUTTON_BEEP_OFF;
	btn -> _state = _state_button (btn);
	btn -> state = btn -> _state;
	btn -> _flag_change = BUTTON_FLAG_RESET;
	ResetButtonFlag (&(btn -> flag_toON));
	ResetButtonFlag (&(btn -> flag_toOFF));
}

ButtonState GetButtonState (TypeButton *btn){
	return (btn -> state);
}

ButtonFlag GetButtonFlag (ButtonFlag *btn_flag){
	ButtonFlag f_btn = *btn_flag;
	*btn_flag = BUTTON_FLAG_RESET;
	return (f_btn);
}

void ResetButtonFlag (ButtonFlag *btn_flag){
	*btn_flag =  BUTTON_FLAG_RESET;
}

void SetButtonBeep (ButtonBeep *beep_to, ButtonBeep beep_stt){
	*beep_to = beep_stt;
}

void Button_Handler (TypeButton *btn){
	ButtonState btn_stt = _state_button (btn);
	if (btn -> _state != btn_stt){
		btn -> _state = btn_stt;
		btn -> cnt_delay = HAL_GetTick();
		btn -> _flag_change = BUTTON_FLAG_SET;
	}
	if ((btn -> _flag_change == BUTTON_FLAG_SET) && (DeltaSysTick (btn -> cnt_delay, TIME_BUTTON_DELAY))){
		btn -> _flag_change = BUTTON_FLAG_RESET;
		if (btn -> _state == btn -> state){
			return; // -------->
		}
		if (btn -> _state == BUTTON_STATE_ON){
			_set_flag_button (btn, &(btn -> flag_toON), &(btn -> beep_toON));
		}
		else{
			_set_flag_button (btn, &(btn -> flag_toOFF), &(btn -> beep_toOFF));
		}
		btn -> state = btn -> _state;
	}
}

/**
  ******************************************************************************
  * @brief   EEPROM functions
  ******************************************************************************
  */
void InitEEPROM (TypeEEPROM *eeprom, I2C_HandleTypeDef *hi2c, uint8_t dev_adr){
	eeprom -> hi2c = hi2c;
	eeprom -> dev_adr = dev_adr;
}

HAL_StatusTypeDef WriteEEPROM (TypeEEPROM *eeprom, uint16_t adr, uint16_t dat){
	HAL_StatusTypeDef status;
	uint8_t dat8[2];
	dat8[0] = dat >> 8;
	dat8[1] = dat & 0xFF;
	HAL_I2C_Mem_Write (eeprom -> hi2c, eeprom -> dev_adr, adr, I2C_MEMADD_SIZE_16BIT, dat8, 2, 100);
	for(;;) {
		status = HAL_I2C_IsDeviceReady(eeprom -> hi2c, eeprom -> dev_adr, 1, 100);
		if(status == HAL_OK){
			break; }
	    }
	return (status);
}

uint16_t ReadEEPROM (TypeEEPROM *eeprom, uint16_t adr){
	uint8_t dat8[2];
	HAL_I2C_Mem_Read (eeprom -> hi2c, eeprom -> dev_adr, adr, I2C_MEMADD_SIZE_16BIT, dat8, 2, 100);
	return (((uint16_t)(dat8[0]) << 8) | (dat8[1] & 0xFF));
}



/**
  ******************************************************************************
  * @brief   Recursive filter functions
  ******************************************************************************
  */
void InitRecFilter   (TypeRecFilter *rfilter, uint16_t dat, uint8_t Kf){
	rfilter -> Kf = Kf;
	rfilter -> buffer = dat / Kf;
}

uint16_t GetRecFiltr (TypeRecFilter *rfilter, uint16_t dat){
	uint16_t res = rfilter -> buffer / rfilter -> Kf;
	rfilter -> buffer += (dat - res);
	return (res);
}

/**
  ******************************************************************************
  * @brief   ADC functions
  ******************************************************************************
  */
static uint16_t _get_adc (TypeADC *adc){
	HAL_ADC_Start(adc -> hadc);
	HAL_ADC_PollForConversion(adc -> hadc,100);
	uint16_t res = ((uint16_t) HAL_ADC_GetValue(adc -> hadc));
	HAL_ADC_Stop(adc -> hadc);
	return (res);
}

void InitADC (TypeADC *adc, ADC_HandleTypeDef *hadc, uint8_t Kf, ADC_Polarity pol){
	adc -> hadc = hadc;
	adc -> pol = pol;
	InitRecFilter   (&(adc -> ADCfilter), _get_adc (adc), Kf);
}

uint16_t GetADC (TypeADC *adc){
	uint16_t  res;
	if (adc -> pol == POLARITY_NORMAL){
		res = GetRecFiltr (&(adc -> ADCfilter), _get_adc (adc));
	}
	else{
		res = 0xFFF - GetRecFiltr (&(adc -> ADCfilter), _get_adc (adc));
	}
	return (res);
}


