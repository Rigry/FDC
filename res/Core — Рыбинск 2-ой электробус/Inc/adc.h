#pragma once


#include "interrupt.h"
#include <cmath>

const uint16_t qty_point{18};

enum {PS = 1, phase_A = 2, phase_C = 3, HV = 4};
enum {V24 = 0, Press = 1, Trad = 2};

class ADC_ : TickSubscriber
{
	Interrupt& adc_callback;
	Interrupt& adc_injected_callback;

	uint8_t qty_channel{0};
	uint16_t time_refresh{1000};

	uint16_t time{0};
	uint16_t buffer[3];

	int16_t arr_current_S[9]{0};
	int16_t arr_current_A[9]{0};
	int16_t arr_current_C[9]{0};
	uint16_t arr_HV[16]{0};
	uint16_t adc_HV{0};
	uint16_t h_voltage{0};
	uint16_t new_hv{0};

	int16_t a{0};
	int16_t b{0};
	int16_t c{0};
	int16_t r{0};
	int16_t new_r{0};

	uint8_t j{0};
	uint8_t m{0};

	bool work{false};

	void adc_interrupt() {
		HAL_ADC_Stop_DMA (&hadc1);
	}

	void adc_injected_interrupt() {

		HAL_ADCEx_InjectedStop_IT (&hadc2);
		arr_current_S[j] = HAL_ADCEx_InjectedGetValue(&hadc2, PS);
		arr_current_A[j] = HAL_ADCEx_InjectedGetValue(&hadc2, phase_A);
		arr_current_C[j] = HAL_ADCEx_InjectedGetValue(&hadc2, phase_C);
		new_hv = HAL_ADCEx_InjectedGetValue(&hadc2, HV);
		h_voltage += (new_hv - h_voltage) * 10 / 60;

//		arr_HV[m] = h_voltage;
//		adc_HV = 0;
//		for (auto i = 0; i <= 15; i++) {
//			adc_HV += arr_HV[i];
//		}
//		adc_HV /= 16;

		if(not work) {
			offset_I_S = 0;
			offset_I_A = 0;
			offset_I_C = 0;
			for (auto i = 0; i < 9; i++) {
				offset_I_S += arr_current_S[i];
				offset_I_A += arr_current_A[i];
				offset_I_C += arr_current_C[i];
			}
			offset_I_S /= (9);
			offset_I_A /= (9);
			offset_I_C /= (9);
		} else if (work) {
			a = (arr_current_A[j] - offset_I_A) * 10 / 21;
			c = (arr_current_C[j] - offset_I_C) * 10 / 21;
			b = -1 * (a + c);

			new_r = (std::sqrt( std::pow((a - b / 2 - c / 2), 2) + std::pow( (b * 17 / 20 - c * 17 / 20), 2) ) * 2) / 3;

			r += (new_r - r) / 4;
		}

		if (j < 8) j++;
		else j = 0;
//		if(m < 15) m++;
//		else m = 0;
	}

	using Parent = ADC_;

	struct ADC_interrupt : Interrupting {
		Parent &parent;
		ADC_interrupt(Parent &parent) :
				parent(parent) {
			parent.adc_callback.subscribe(this);
		}
		void interrupt() override {
			parent.adc_interrupt();
		}
	} adc_ { *this };

	struct ADC_INJ_interrupt : Interrupting {
		Parent &parent;
		ADC_INJ_interrupt(Parent &parent) :
				parent(parent) {
			parent.adc_injected_callback.subscribe(this);
		}
		void interrupt() override {
			parent.adc_injected_interrupt();
		}
	} adc_injected_ { *this };

public:

	ADC_(Interrupt& adc_callback, Interrupt& adc_injected_callback, uint8_t qty_channel, uint16_t time_refresh)
    : adc_callback {adc_callback}
    , adc_injected_callback {adc_injected_callback}
    , qty_channel  {qty_channel}
    , time_refresh {time_refresh}
	{
		subscribed = false;
		if (time_refresh > 0)
		  subscribe();
	}

	int16_t offset_I_S{0};
	int16_t offset_I_A{0};
	int16_t offset_I_C{0};

	void measure_offset() {
		work = false;
	}

	void measure_value() {
		work = true;
	}

	uint16_t operator[](uint8_t i) {
		return buffer[i];
	}

	uint16_t value(uint8_t i) {
		if (i == PS)
			return arr_current_S[j];
		if (i == phase_A)
			return arr_current_A[j];
		if (i == phase_C)
			return arr_current_C[j];
		if (i == HV)
			return h_voltage;
	}

	uint16_t current(){
		return r;
	}


	void notify(){
		if (time++ >= time_refresh) {
		   time = 0;
		   HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer, qty_channel);
		}
		if( not time % 100 and not work)
			HAL_ADCEx_InjectedStart_IT(&hadc2);
	}


};

Interrupt adc_callback;
Interrupt adc_injected_callback;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc){
	if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
	{
		adc_callback.interrupt();
	}
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc->Instance == ADC2) //check if the interrupt comes from ACD2
	{
		adc_injected_callback.interrupt();
	}
}

