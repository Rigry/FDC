#pragma once

class Cooler {

	Service<In_data, Out_data>& service;
	uint16_t CCR_value{0};

public:

	Cooler(Service<In_data, Out_data>& service) : service{service}{}

	void operator()(){

		if (service.outData.convertor_temp >= 40) {
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			CCR_value = 899 + 90 * (service.outData.convertor_temp - 40);
			TIM3->CCR1 = CCR_value;
		} else {
			TIM3->CCR1 = 0;
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		}

	}

};
