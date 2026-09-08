#include "adc_input.h"
#include "adc.h"

static ADC_InputData adc_input_data;

HAL_StatusTypeDef ADC_Input_Update(void)
{
  HAL_StatusTypeDef status;
  uint16_t horizontal_raw;
  uint16_t pitch_raw;

  status = HAL_ADC_Start(&hadc1);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADC_PollForConversion(&hadc1, 2U);
  if (status != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return status;
  }
  horizontal_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);

  status = HAL_ADC_PollForConversion(&hadc1, 2U);
  if (status != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return status;
  }
  pitch_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);

  status = HAL_ADC_Stop(&hadc1);
  if (status == HAL_OK)
  {
    adc_input_data.horizontal_raw = horizontal_raw;
    adc_input_data.pitch_raw = pitch_raw;
  }

  return status;
}

void ADC_Input_Get(ADC_InputData *data)
{
  if (data != NULL)
  {
    *data = adc_input_data;
  }
}
