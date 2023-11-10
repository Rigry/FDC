#pragma once

#include "adc.h"
#include "service.h"
#include "contactor.h"
#include "interrupt.h"
#include "pin.h"

class Convertor {

	enum {SYNCHRON = false, ASYNCHRON = true};
	enum State {wait, starting} state{wait};

	ADC_& adc;
	Service<In_data, Out_data>& service;
	Contactor& contactor;
	Interrupt& period_callback;
	Interrupt& adc_comparator_callback;
	Pin& led_red;
	Pin& led_green;
	Pin& unload;
	Pin& condens;
	Pin& TD_DM;
	Pin& SP;
	Pin& Start;
	Pin& Motor;

	Timer timer;
	Timer rerun;
	Timer delay;
	Timer rotation;
	Timer timer_stop;
	Timer restart;
	bool rotation_{false};

	uint16_t sin_table[qty_point]{3600, 5514, 6764
								, 7199, 6764, 6764
								, 7199, 6764, 5514
								, 3600, 1250,    0
								,    0,    0,    0
								,    0,    0, 1250
								};
	uint8_t r{0};
	uint8_t k{0};
	uint8_t m{6};
	uint8_t n{12};
	uint32_t Km{5};
	uint16_t Kp{1150};
	uint16_t frequency{100};
	uint16_t max_current{25};
	uint16_t need_current{4000};
	int16_t e{0};
	uint16_t time{2};
	uint16_t U_phase{0};
	bool U_stop{false};
	bool motor{false};
	uint16_t U_phase_max{0};
	uint8_t offset{25};
	uint8_t error{0};
	uint8_t error_S{0};
	uint8_t error_A{0};
	uint8_t error_C{0};
	uint16_t min_ARR{356};

	bool enable{true};

//	float radian = 10 * 3.14 / 180;
	uint32_t div_f = 1'800'000 / (qty_point);

	using Parent = Convertor;

	struct TIM3_interrupt: Interrupting {
		Parent &parent;
		TIM3_interrupt(Parent &parent) :
				parent(parent) {
			parent.period_callback.subscribe(this);
		}
		void interrupt() override {
			parent.period_interrupt();
		}
	} tim3_interrupt { *this };

	struct adc_comparator_interrupt: Interrupting {
		Parent &parent;
		adc_comparator_interrupt(Parent &parent) :
				parent(parent) {
			parent.adc_comparator_callback.subscribe(this);
		}
		void interrupt() override {
			parent.comparator_interrupt();
		}
	} adc_comparator_ { *this };

	void period_interrupt(){

		TIM1->CCR1 = Km * sin_table[k++] / 1000;
		TIM1->CCR2 = Km * sin_table[m++] / 1000;
		TIM1->CCR3 = Km * sin_table[n++] / 1000;

		if (k >= qty_point) k = 0;
		if (m >= qty_point) m = 0;
		if (n >= qty_point) n = 0;

//		TIM3->ARR = (div_f / (frequency)) * 10 - 1;

		HAL_ADCEx_InjectedStart_IT(&hadc2);

	}

	void comparator_interrupt() {

	}

public:

	Convertor(ADC_& adc, Service<In_data, Out_data>& service, Contactor& contactor, Interrupt& period_callback, Interrupt& adc_comparator_callback
			, Pin& led_red, Pin& led_green, Pin& unload, Pin& condens, Pin& TD_DM, Pin& SP, Pin& Start, Pin& Motor)
	: adc{adc}, service{service}, contactor{contactor}, period_callback{period_callback}, adc_comparator_callback{adc_comparator_callback}
	, led_red{led_red}, led_green{led_green}, unload{unload}, condens{condens}, TD_DM{TD_DM}, SP{SP}, Start{Start}, Motor{Motor}
	{rerun.time_set = 0; delay.time_set = 0; rotation.time_set = 0; timer_stop.time_set = 0; restart.time_set = 0;}

	void operator() (){

		contactor();
		service();

		service.outData.PWM = Km;
		service.outData.error.on = Start;
		service.outData.U_phase = U_phase;
		service.outData.error.overheat_c = not bool(TD_DM);
		service.outData.error.HV_low = /*(service.outData.high_voltage <= 300) or*/ U_stop;
		service.outData.error.voltage_board_low = (service.outData.voltage_board <= 18);
//		service.outData.error.voltage_board_low = false;
		service.outData.error.HV = service.outData.high_voltage >= 850;
		service.outData.max_current_A = min_ARR;
		service.outData.max_current_C = U_phase_max;

		service.outData.voltage_board = Kp;
		service.outData.max_current = TIM3->ARR;

		if(service.outData.high_voltage < 300) U_stop = true;
		else if(service.outData.high_voltage > 400) U_stop = false;

		if (service.outData.error.overheat_fc |= service.outData.convertor_temp >= 60) {
			service.outData.error.overheat_fc = service.outData.convertor_temp >= 50;
		}

		if(contactor.is_on() and enable) alarm();

		switch(state) {
		case wait:
			motor = Motor;

			if(delay.done()) {
				delay.stop();
			}

if(motor == ASYNCHRON) {
	/*
	if (service.outData.high_voltage > 400 and service.outData.high_voltage < 540) {
		U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
		min_ARR = (div_f / ((U_phase_max) * 9)) * 22; // 5/22 = 50/220
	} else {
		U_phase_max = 220;
		min_ARR = 1100;
	}

*/
			unload = false;
			if (service.outData.high_voltage > 400 and service.outData.high_voltage < 540) {
				U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
				min_ARR = (div_f / ((U_phase_max) * 5)) * 22; // 5/22 = 50/220
			} else {
				U_phase_max = 220;
				min_ARR = 2000;
			}

} else if (motor == SYNCHRON) {
			unload = true;
			if(service.outData.high_voltage > 400 and service.outData.high_voltage < 540) {
				U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
				min_ARR = ((div_f / (U_phase_max)) * 55) / 70; // 70/53 = 280/212
				if(min_ARR < 357) min_ARR = 357;
			} else {
				U_phase_max = 220;
				min_ARR = 356;
			}
}

			enable = Start and not rerun.isCount() /*and not service.pressure_is_normal()*/ and not delay.isCount()
					 and not service.outData.error.overheat_fc and not service.outData.error.overheat_c
					 and not service.outData.error.HV and not service.outData.error.HV_low
					 and not service.outData.error.voltage_board_low and (error < 3) and not U_stop;

			if(rerun.done()) rerun.stop();

			if(error >= 3 and not restart.isCount()) {
				restart.start(60'000);
			}

			if(error >= 3 and restart.done()) {
				restart.stop();
				error = 0;
			}

			if (enable){
				rerun.stop();
				contactor.start();
				if(contactor.is_on()) {
					pusk();
					state = State::starting;
				}
			} else stop();

			if (not Start) {
//				U_stop = false;
				rerun.stop();
				rerun.time_set = 0;
				restart.stop();
				restart.time_set = 0;
				error = 0;
				led_red = false;
			}

			break;
		case starting:

if(motor == ASYNCHRON) {

	/*
	if (service.outData.high_voltage > 400 and service.outData.high_voltage < 540) {
		U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
		min_ARR = (div_f / ((U_phase_max) * 9)) * 22; // 5/22 = 50/220
	} else {
		U_phase_max = 220;
		min_ARR = 1100;
	}

	U_phase = ((((service.outData.high_voltage / 20) * Km) / 141) * 112) / 100; // 31 = 620 / 20; 141 = sqrt(2) * 100; 115 = добавочный
	Km = offset + ((Kp * (div_f / TIM3->ARR) / (service.outData.high_voltage)) * 4 )/ 3;

	if (TIM3->ARR <= (min_ARR + 10)) {
		error = 0;
	}

	if (Kp > 12000) {
		Kp = 12000;
	}

	if (TIM3->ARR <= min_ARR) {
		if (U_phase > U_phase_max) {
				Kp--;
		} else {
		if (adc.current() < 120)
				Kp++;
		}

		if (adc.current() > 160) {
			if (Kp > 5000) {
				Kp -= 4;
			}
		}
	}

	if (adc.current() < 35) {
		if (Kp < 12000) {
			Kp++;
		}
	}

	if (TIM3->ARR > (min_ARR + 5)) {
		if (adc.current() > 75) {
			if (Kp >= 6000) {
				Kp--;
			}
		}
	}

*/
			if (service.outData.high_voltage > 300 and service.outData.high_voltage < 540) {
				U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
				min_ARR = (div_f / ((U_phase_max) * 5)) * 22; // 5/22 = 50/220
			} else {
				U_phase_max = 220;
				min_ARR = 2000;
			}

			U_phase = ((((service.outData.high_voltage / 20) * Km) / 141) * 112) / 100; // 31 = 620 / 20; 141 = sqrt(2) * 100; 115 = добавочный
			Km = offset + ((Kp * (div_f / TIM3->ARR) / (service.outData.high_voltage)) * 4 )/ 3;

			if (TIM3->ARR <= (min_ARR + 10)) {
				unload = true;
				error = 0;
			}

			if (Kp > 12000) {
				Kp = 12000;
			}

			if (TIM3->ARR <= min_ARR) {
				if (U_phase > U_phase_max) {
					Kp--;
				} else {
					if(adc.current() < 120)
					Kp++;
				}

				if (adc.current() > 160) {
					if (Kp > 5000) {
						Kp -= 4;
					}
				}
			}

			if (adc.current() < 35) {
				if (Kp < 12000) {
					Kp++;
				}
			}

			if (TIM3->ARR > (min_ARR + 5)) {
				if (adc.current() > 75) {
					if (Kp >= 6000) {
						Kp--;
					}
				}
			}


} else if(motor == SYNCHRON) {
				if (service.outData.high_voltage > 300 and service.outData.high_voltage < 540) {
					U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
					min_ARR = ((div_f / (U_phase_max)) * 55) / 70; // 70/53 = 280/212
					if(min_ARR < 357) min_ARR = 357;
				} else {
					min_ARR = 356;
					U_phase_max = 220;
				}

				U_phase = ((((service.outData.high_voltage / 20) * Km) / 141) * 112) / 100; // 31 = 620 / 20; 141 = sqrt(2) * 100; 115 = добавочный
				Km = offset + Kp * (div_f / TIM3->ARR) / (service.outData.high_voltage);



				if (TIM3->ARR <= (min_ARR + 10)) {
					unload = false;
					error = 0;
				}

				if (Kp > 2200) {
					Kp = 2200;
				}

				if(TIM3->ARR <= min_ARR) {
					if (U_phase > U_phase_max) {
						Kp--;
					} else {
						Kp++;
					}

					if (adc.current() > 160) {
						if (Kp > 1250) {
							Kp -= 4;
						}
					}
				}
				if (adc.current() < 35) {
					if (Kp < 2200) {
						Kp++;
					}
				}
				if (TIM3->ARR > (min_ARR + 5)) {
					if (adc.current() > 110) {
						if(Kp > 1250) {
							Kp--;
						}
					}
				}

} //else if(motor == SYNCHRON) {

			if (Km >= 990) {
				Km = 990;
			}

			if (timer.done()) {
				timer.stop();
				timer.start(time);

if(motor == ASYNCHRON) {

	if (not rotation.isCount() and TIM3->ARR != min_ARR) {
		if (TIM3->ARR > 6000) {
			TIM3->ARR -= 10;
		} else if (TIM3->ARR > min_ARR) {
			TIM3->ARR -= 4;
		} else {
			TIM3->ARR++;
		}
	}

} else if(motor == SYNCHRON) {
			if(TIM3->ARR != min_ARR) {
				if(TIM3->ARR > min_ARR) {
					if(TIM3->ARR > 624) {
						if(TIM3->ARR > 1500) {
							TIM3->ARR -= 20;
						} else {

							TIM3->ARR -= 2;
						}
					} else {
						TIM3->ARR--;
					}
				} else {
					TIM3->ARR++;
				}

				if(TIM3->ARR > 624) {
					time = 2;
				} else if (TIM3->ARR > 554) {
					time = 8;
				} else if (TIM3->ARR < 554) {
					time = 8;
				}

			}
	}
} // else if(motor == SYNCHRON) {
			break;
		}
	}

	void pusk() {

if(motor == ASYNCHRON) {
		frequency = 60;
		Kp = 6000;
		time = 3;
		offset = 25;

} else if(motor == SYNCHRON) {
		frequency = 10;
		Kp = 1150;
		time = 2;
		offset = 25;
} // else if(motor == SYNCHRON) {

		Km = 5;
		TIM3->ARR = (div_f / (frequency)) * 10 - 1;

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

		HAL_TIM_Base_Start_IT(&htim3);

		timer.start(time);
		adc.measure_value();

		service.outData.error.current_S = false;
		service.outData.error.current_A = false;
		service.outData.error.current_C = false;
	}

	void stop() {

		TIM1->CCR1 = 0;
		TIM1->CCR1 = 0;
		TIM1->CCR1 = 0;
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

		HAL_TIM_Base_Stop_IT(&htim3);
		timer.stop();
		contactor.stop();
		rotation_ = false;

		k = 0;
		m = 6;
		n = 12;

//		TIM3->ARR = (div_f / (frequency)) * 10 - 1;

		state = State::wait;

		adc.measure_offset();

	}

	void alarm() {
		if((not Start or timer_stop.done()) or not contactor.is_on() /*or service.pressure_is_normal()*/
				     or service.outData.error.overheat_fc or service.outData.error.overheat_c
				     or service.outData.error.HV_low or service.outData.error.HV or service.outData.error.voltage_board_low
					 )
		{
//			if(service.pressure_is_normal()) error = 0;
			if(not Start and not timer_stop.isCount()) {
				timer_stop.start(1000);
			}

			if(timer_stop.done() and not Start) {
				stop();
				timer_stop.stop();
//				if (motor == ASYNCHRON and not delay.isCount()) {
//					delay.start(90'000);
//				}
			}

			if( not contactor.is_on() /*or service.pressure_is_normal()*/
				     or service.outData.error.overheat_fc or service.outData.error.overheat_c
				     or service.outData.error.HV_low or service.outData.error.HV or service.outData.error.voltage_board_low) {
				stop();
				timer_stop.stop();
//				if (motor == ASYNCHRON and not delay.isCount()) {
//					delay.start(90'000);
//				}
			}

		}
		if (abs(adc.value(PS) - adc.offset_I_S) / 21 > 16 and not service.outData.error.current_S and adc.is_measure()) {
			error_S++;
			if(error_S >= 5) {
				error++;
				led_red = true;
				stop();
				service.outData.error.current_S = true;
				rerun.start(3500);
			}

		} else {
			error_S = 0;
		}
		if (abs(adc.value(phase_A) - adc.offset_I_A) / 21 > max_current and not service.outData.error.current_A and adc.is_measure()) {
//			error++;
			error_A++;
			if(error_A >= 5) {
				error++;
				led_red = true;
				stop();
				service.outData.error.current_A = true;
				rerun.start(3500);
			}

		} else {
			error_A = 0;
		}

		if (abs(adc.value(phase_C) - adc.offset_I_C) / 21 > max_current and not service.outData.error.current_C and adc.is_measure()) {
//			error++;
//			service.outData.error.current_C = true;
			error_C++;
			if(error_C >= 5) {
				error++;
				led_red = true;
				stop();
				service.outData.error.current_C = true;
				rerun.start(3500);
			}
		} else {
			error_C = 0;
		}
		adc.reset_measure();
	}

};

Interrupt period_callback;
Interrupt adc_comparator_callback;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
	if(htim->Instance == TIM3) //check if the interrupt comes from ACD2
	{
		period_callback.interrupt();
	}
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc){
	if(hadc->Instance == ADC2) //check if the interrupt comes from ACD2
	{
		adc_comparator_callback.interrupt();
	}
}



//timer.stop();
//timer.start(15);
//if((service.outData.current_A * 100) < (need_current + 200)) {
//	e = (need_current - (service.outData.current_A * 100)) / 1500;
//	Km += e;
//}
//else {
//	state = State::overclocking;
//	Kp = Km;
//}
//if(Km > 100) Km = 100;

//				if(TIM3->ARR < 1999 and offset) {
//
//					if(en) {
//						offset--;
//						en = false;
//					} else {
//						en = true;
//					}
//
//				}


