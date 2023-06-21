#include "PulseOut.h"
#include <cstdint>

#if defined(TARGET_STM) && !defined(TARGET_STM32F0) && !defined(TARGET_STM32L0) && !defined(TARGET_STM32L1)

#include <pwmout_device.h>

extern const PinMap PinMap_PWM[];
extern const pwm_apb_map_t pwm_apb_map_table[];

OPMPulseOut::OPMPulseOut(PinName pin)
{
  sleep_manager_lock_deep_sleep();

  int peripheral = (int)pinmap_peripheral(pin, PinMap_PWM);
  int function = (int)pinmap_find_function(pin, PinMap_PWM);

  const PinMap pinmap = {pin, peripheral, function};
  obj.pwm = (PWMName)pinmap.peripheral;

  // Get the functions (timer channel, (non)inverted) from the pin and assign it to the object
  obj.channel = STM_PIN_CHANNEL(function);
  obj.inverted = STM_PIN_INVERTED(function);

  // Enable TIM clock
  bool clock_enabled = false;
#if defined(TIM1_BASE)
  if (obj.pwm == PWM_1) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    clock_enabled = true;
  }
#endif
#if defined(TIM8_BASE)
  if (obj.pwm == PWM_8) {
      printf("Enabling TIM8\n");
    __HAL_RCC_TIM8_CLK_ENABLE();
    clock_enabled = true;
  }
#endif
#if defined(TIM15_BASE)
  if (obj.pwm == PWM_15) {
    __HAL_RCC_TIM15_CLK_ENABLE();
    clock_enabled = true;
  }
#endif
#if defined(TIM16_BASE)
  if (obj.pwm == PWM_16) {
    __HAL_RCC_TIM16_CLK_ENABLE();
    clock_enabled = true;
  }
#endif
#if defined(TIM17_BASE)
  if (obj.pwm == PWM_17) {
    __HAL_RCC_TIM17_CLK_ENABLE();
    clock_enabled = true;
  }
#endif
#if defined(TIM20_BASE)
  if (obj.pwm == PWM_20) {
    __HAL_RCC_TIM20_CLK_ENABLE();
    clock_enabled = true;
  }
#endif

  if(!clock_enabled)
  {
    error("PulseOut only supports TIM1,TIM8,TIM15,TIM16,TIM17 and TIM20\n");
  }

  // Configure GPIO
  pin_function(pinmap.pin, pinmap.function);

  obj.pin = pinmap.pin;
  obj.period = 0;
  obj.pulse = 0;
  obj.prescaler = 1;
}

OPMPulseOut::~OPMPulseOut()
{
    sleep_manager_unlock_deep_sleep();
    pwmout_free(&obj);
}

void OPMPulseOut::write_us(int period_us, int width_us, int count, int batch_size)
{
    int current_batch_count = count > batch_size ? batch_size: count;
    write_once(period_us, width_us, current_batch_count);
    count -= current_batch_count;
    wait();
    while(count > 0)
    {
        current_batch_count = count > batch_size ? batch_size: count;
        write_more(current_batch_count);
        count -= current_batch_count;
        wait();
    }
}

void OPMPulseOut::write_once(int period_us, int width_us, int count)
{
  TIM_Handler.Instance = (TIM_TypeDef *)(obj.pwm);
  __HAL_TIM_DISABLE(&TIM_Handler);

  // Get clock configuration
  // Note: PclkFreq contains here the Latency (not used after)
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  uint32_t PclkFreq = 0;
  uint32_t APBxCLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &PclkFreq);

  uint8_t i = 0;
  while (pwm_apb_map_table[i].pwm != obj.pwm) {
    i++;
  }

  if (pwm_apb_map_table[i].pwm == 0) {
    error("Unknown PWM instance\n");
  }

  if (pwm_apb_map_table[i].pwmoutApb == PWMOUT_ON_APB1) {
    PclkFreq = HAL_RCC_GetPCLK1Freq();
    APBxCLKDivider = RCC_ClkInitStruct.APB1CLKDivider;
  } else {
#if !defined(PWMOUT_APB2_NOT_SUPPORTED)
    PclkFreq = HAL_RCC_GetPCLK2Freq();
    APBxCLKDivider = RCC_ClkInitStruct.APB2CLKDivider;
#endif
  }  

  /* By default use, 1us as SW pre-scaler */
  obj.prescaler = 1;
  // TIMxCLK = PCLKx when the APB prescaler = 1 else TIMxCLK = 2 * PCLKx
  if (APBxCLKDivider == RCC_HCLK_DIV1) {
    TIM_Handler.Init.Prescaler = (((PclkFreq) / 1000000)) - 1; // 1 us tick
  } else {
    TIM_Handler.Init.Prescaler = (((PclkFreq * 2) / 1000000)) - 1; // 1 us tick
  }
  TIM_Handler.Init.Period = (period_us - 1);
  /*  In case period or pre-scalers are out of range, loop-in to get valid values */
  while ((TIM_Handler.Init.Period > 0xFFFF) || (TIM_Handler.Init.Prescaler > 0xFFFF)) {
    obj.prescaler = obj.prescaler * 2;
    if (APBxCLKDivider == RCC_HCLK_DIV1) {
      TIM_Handler.Init.Prescaler = (((PclkFreq) / 1000000) * obj.prescaler) - 1;
    } else {
      TIM_Handler.Init.Prescaler = (((PclkFreq * 2) / 1000000) * obj.prescaler) - 1;
    }
    TIM_Handler.Init.Period = (period_us - 1) / obj.prescaler;
    /*  Period decreases and prescaler increases over loops, so check for
     *  possible out of range cases */
    if ((TIM_Handler.Init.Period < 0xFFFF) && (TIM_Handler.Init.Prescaler > 0xFFFF)) {
      error("Cannot initialize PWM\n");
      break;
    }
  }

  TIM_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; 
  TIM_Handler.Init.RepetitionCounter = count - 1;

  if(HAL_TIM_OnePulse_Init(&TIM_Handler, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    error("HAL_TIM_OnePulse_Init failed\n");
  }

  TIM_OC_InitTypeDef sConfig;
  sConfig.OCMode     = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_LOW;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.Pulse = (period_us - width_us) * (TIM_Handler.Init.Period + 1) / period_us;

  int channel = 0;
  switch (obj.channel) {
    case 1:
      channel = TIM_CHANNEL_1;
      break;
    case 2:
      channel = TIM_CHANNEL_2;
      break;
    default:
      error("Only Channe 1&2 supports OPM\n");
      return;
  }

  if (HAL_TIM_PWM_ConfigChannel(&TIM_Handler, &sConfig, channel) != HAL_OK)
  {
    error("HAL_TIM_PWM_ConfigChannel failed\n");
  }

  if (obj.inverted)
  {
    if(HAL_TIMEx_PWMN_Start(&TIM_Handler, channel) != HAL_OK)
    {
      error("HAL_TIM_PWN_Start failed\n");
    }
  }
  else
  {
    if (HAL_TIM_PWM_Start(&TIM_Handler, channel) != HAL_OK)
    {
      error("HAL_TIM_PWM_Start failed\n");
    }
  }
}

void OPMPulseOut::write_more(int count)
{
  TIM_Handler.Instance->RCR = uint32_t(count - 1);
  TIM_Handler.Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_ENABLE(&TIM_Handler);
}

void OPMPulseOut::wait()
{
    while((TIM_Handler.Instance->CR1 & TIM_CR1_CEN_Msk) == TIM_CR1_CEN)
    {
        ThisThread::yield();
    }
}

#endif

SingletonPtr<TickerPulseOut> ticker_pulse_out_ptr;

TickerPulseOut* TickerPulseOut::get_instance()
{
    return ticker_pulse_out_ptr.get();
}

void TickerPulseOut::ticker_callback()
{
    bool has_pulses = false;
    #pragma unroll
    for(int i = 0; i < PULSEOUT_MAX_PULSE_COUNT; i++)
    {
        if(pulses[i].count > 0)
        {
            pulses[i].phase += PULSEOUT_TICKER_INTERVAL_US;
            if(pulses[i].phase >= pulses[i].period_us)
            {
                pulses[i].phase = 0;
                pulses[i].count--;
            }
            if(pulses[i].count > 0)
            {
                if(pulses[i].phase == 0)
                {
                    pulses[i].out.write(1);
                }
                else if(pulses[i].phase >= pulses[i].width_us && 
                    pulses[i].phase < pulses[i].width_us + PULSEOUT_TICKER_INTERVAL_US)
                {
                    pulses[i].out.write(0);
                }
                has_pulses = true;
            }
        }
    }
    if(!has_pulses)
    {
        ticker.detach();
        ticker_running = false;
    }
}

bool TickerPulseOut::write_us(PinName pin, int period_us, int width_us, int count)
{
    CriticalSectionLock critical;
    for(int i = 0; i < PULSEOUT_MAX_PULSE_COUNT; i++)
    {
        if(pulses[i].count <= 0)
        {
            pulses[i].out.~DigitalOut();
            new (&pulses[i].out) DigitalOut(pin);
            pulses[i].pin = pin;
            pulses[i].period_us = period_us;
            pulses[i].width_us = width_us;
            pulses[i].count = count;
            pulses[i].phase = -PULSEOUT_TICKER_INTERVAL_US;
            pulses[i].out.write(0);
            if(!ticker_running)
            {
                ticker.attach(callback(this, &TickerPulseOut::ticker_callback), chrono::microseconds(PULSEOUT_TICKER_INTERVAL_US));
                ticker_running = true;
            }
            return true;
        }
    }
    return false;
}
