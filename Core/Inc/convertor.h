#pragma once

#include "adc.h"
#include "service.h"
#include "contactor.h"
#include "interrupt.h"
#include "pin.h"

#define CONDITIONER

class Convertor {

	enum {SYNCHRON = false, ASYNCHRON = true};
	enum State {wait, starting} state{wait};

	ADC_& adc;
	Service<In_data, Out_data>& service;
	Contactor& contactor;
	Interrupt& period_callback;
//	Interrupt& adc_comparator_callback;
	Pin& led_red;
	Pin& led_green;
	Pin& ventilator;
	Pin& unload;
	Pin& condens;
	Pin& TD_DM;
	Pin& SP;
	Pin& Start;
	Pin& Motor;

	Timer timer;
	Timer rerun;
	Timer timer_stop;
	Timer clump_timer;
	Timer work;
	Timer rest;
	Timer blink;

//	uint16_t sin_table[qty_point]{3600, 5514, 6764
//								, 7199, 6764, 6764
//								, 7199, 6764, 5514
//								, 3600, 1250,    0
//								,    0,    0,    0
//								,    0,    0, 1250
//								};
	uint16_t sin_table[qty_point]{3600, 4627, 5514, 6234, 6764, 7089
								, 7199, 7089, 6764, 6234, 6764, 7089
								, 7199, 7089, 6764, 6234, 5514, 4627
								, 3600, 2462, 1250,    0,    0,    0
								,    0,    0,    0,    0,    0,    0
								,    0,    0,    0,    0, 1250, 2462
								};

//	uint16_t sin_table[qty_point]{3599, 4223, 4829, 5398, 5912, 6355
//								, 6715, 6980, 7143, 7198, 7143, 6980
//								, 6715, 6355, 5912, 5398, 4829, 4223
//								, 3599, 2974, 2368, 1799, 1285,  842
//								,  482,  217,   54,    0,   54,  217
//								,  482,  842, 1285, 1799, 2368, 2974
//								};

	uint8_t r{0};
	uint8_t k{0};
	uint8_t m{12};
	uint8_t n{24};
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
	uint8_t error_F{0};
	uint16_t min_ARR{360};
	uint16_t value_ARR{380};
	uint16_t ARR_ASIN{2000};
	uint16_t ARR_CONDIT{1333};

	bool enable{true};
	bool phase{false};
	bool cool{false};
	bool cold{false};
	bool switcher{false};

//	float radian = 10 * 3.14 / 180;
	uint32_t div_f = 3'600'000 / (qty_point);

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

//	struct adc_comparator_interrupt: Interrupting {
//		Parent &parent;
//		adc_comparator_interrupt(Parent &parent) :
//				parent(parent) {
//			parent.adc_comparator_callback.subscribe(this);
//		}
//		void interrupt() override {
//			parent.comparator_interrupt();
//		}
//	} adc_comparator_ { *this };

	void period_interrupt(){

		if (Km >= 750) {
			Km = 750;
		}

		TIM1->CCR1 = Km * sin_table[m++] / 1000;
		TIM1->CCR2 = Km * sin_table[k++] / 1000;
		TIM1->CCR3 = Km * sin_table[n++] / 1000;

		if (k >= qty_point) {k = 0;}
		if (m >= qty_point) {m = 0;}
		if (n >= qty_point) {n = 0;}

		switcher ^= 1;

		if(switcher) HAL_ADCEx_InjectedStart_IT(&hadc2);

	}

//	void comparator_interrupt() {
//
//	}

public:

	Convertor(ADC_& adc, Service<In_data, Out_data>& service, Contactor& contactor, Interrupt& period_callback/*, Interrupt& adc_comparator_callback*/
			, Pin& led_red, Pin& led_green, Pin& ventilator, Pin& unload, Pin& condens, Pin& TD_DM, Pin& SP, Pin& Start, Pin& Motor)
	: adc{adc}, service{service}, contactor{contactor}, period_callback{period_callback}/*, adc_comparator_callback{adc_comparator_callback}*/
	, led_red{led_red}, led_green{led_green}, ventilator{ventilator}, unload{unload}, condens{condens}, TD_DM{TD_DM}, SP{SP}, Start{Start}, Motor{Motor}
	{rerun.time_set = 0; timer_stop.time_set = 0; clump_timer.time_set = 0;
		if(motor == SYNCHRON) {
			unload = true;
			clump_timer.start(15000);
		}
		motor = Motor;
	}

	void operator() (){

		service();
		contactor();

		service.outData.PWM = Km;
		service.outData.error.on = Start;
		service.outData.U_phase = U_phase;
		service.outData.error.overheat_c = not bool(TD_DM);
		service.outData.error.HV_low = U_stop;
		service.outData.error.voltage_board_low = (service.outData.voltage_board < 180);
		service.outData.error.voltage_board_high = (service.outData.voltage_board > 300);

		service.outData.max_current_A = min_ARR;
		service.outData.max_current_C = U_phase_max;
		service.outData.current_C = Kp;
		service.outData.max_current = TIM3->ARR;

		if(service.outData.high_voltage <= 300) U_stop = true;
		else if(service.outData.high_voltage > 310) {U_stop = false; adc.reset_error_HV();}

		if (service.outData.error.overheat_fc |= service.outData.convertor_temp >= 75) {
			service.outData.error.overheat_fc = service.outData.convertor_temp >= 70;
		}

#ifdef CONDITIONER
///////////////CONDITIONER
		if (cool |= service.outData.convertor_temp >= 35) {
			cool = service.outData.convertor_temp >= 30;
		}

//		if(enable)
			ventilator = cool;
//		else
//			ventilator = false;
///////////////CONDITIONER
#endif
		if(contactor.is_on() and enable) alarm();

		switch(state) {
		case wait:

if(motor == ASYNCHRON) {

#ifdef CONDITIONER
/////////////////CONDITIONER
	adc.set_max_current(14);
	adc.set_max_current_phase(18);
	if (service.outData.high_voltage > 300 and service.outData.high_voltage < 400) {
		U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 112) / 100;
		min_ARR = (div_f / ((U_phase_max) * 9)) * 22; // 5/22 = 50/220
		if(min_ARR <= ARR_CONDIT) min_ARR = ARR_CONDIT;
	} else {
		U_phase_max = 180;
		min_ARR = ARR_CONDIT;
	}
/////////////////CONDITIONER
#else
	adc.set_max_current(16);
	adc.set_max_current_phase(20);
	unload = false;
	if (service.outData.high_voltage > 300 and service.outData.high_voltage < 540) {
		U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
		min_ARR = (div_f / ((U_phase_max) * 5)) * 22; // 5/22 = 50/220
		if(min_ARR <= ARR_ASIN) min_ARR = ARR_ASIN;
	} else {
		U_phase_max = 220;
		min_ARR = ARR_ASIN;
	}
#endif

} else if (motor == SYNCHRON) {

	adc.set_max_current(16);
	adc.set_max_current_phase(20);
	if(clump_timer.done()) {
		clump_timer.stop(); unload = false;
	}
 	if(service.outData.high_voltage > 300 and service.outData.high_voltage < 540) {
		U_phase_max = ((((service.outData.high_voltage / 20) * 940) / 141) * 115) / 100;
		min_ARR = ( (div_f / (U_phase_max)) * 50) / 70; // 70/53 = 280/212
		if(min_ARR < value_ARR) min_ARR = value_ARR;
	} else {
		U_phase_max = 212;
		min_ARR = value_ARR;
	}

}

			enable = Start
					 and not rerun.isCount() and not rest.isCount()
					 and not service.outData.error.overheat_fc /*and not service.outData.error.overheat_c*/
					 and not service.outData.error.voltage_board_low and not service.outData.error.voltage_board_high
					 and not U_stop;

			if(rerun.done()) rerun.stop();
			if(rest.done()) rest.stop();
			if (blink.done() and rest.isCount()) {
				blink.stop();
				blink.start(300);
				led_red ^= true;
				led_green = not led_red;
			}

			if (enable){
				rerun.stop();
				contactor.start();
				if(contactor.is_on()) {
					pusk();
					state = State::starting;
				}
			}

			if (not Start) {
				rerun.stop();
				rerun.time_set = 0;
//				led_red = false;
				adc.reset_error();
				phase = false;
			}

			break;
		case starting:

			adc.what_Km(Km);

if(motor == ASYNCHRON) {

#ifdef CONDITIONER
/////////////////CONDITIONER
	if (service.outData.high_voltage > 300 and service.outData.high_voltage < 400) {
		U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
		min_ARR = (div_f / ((U_phase_max) * 9)) * 22; // 5/22 = 50/220
		if(min_ARR <= ARR_CONDIT) min_ARR = ARR_CONDIT;
	} else {
		U_phase_max = 180;
		min_ARR = ARR_CONDIT;
	}

//	U_phase = ((((service.outData.high_voltage / 20) * Km) / 141) * 112) / 100; // 31 = 620 / 20; 141 = sqrt(2) * 100; 115 = добавочный
	Km = offset + ( ((Kp * (div_f / TIM3->ARR) / (service.outData.high_voltage + 1))) );

	if (TIM3->ARR <= uint32_t(min_ARR + 10)) {
		error = 0;
	}

	if (Kp >= 5500) {
		Kp = 5500;
	}

	if (TIM3->ARR <= min_ARR) {
//		if (U_phase - U_phase_max > 10) {
//			Kp--;
//		} else {
//			if((U_phase_max - U_phase > 10))
//				Kp++;
//		}

		if (adc.current() > 180) {
			if (Kp > 4500) {
				Kp -= 4;
			}
		}
	}

	if (adc.current() < 140) {
		if (Kp < 5500) {
			Kp++;
		}
	}

	if (TIM3->ARR > uint32_t(min_ARR + 5)) {
		if (adc.current() > 100) {
			if (Kp >= 5000) {
				Kp--;
			}
		}
	}
/////////////////CONDITIONER
#else

	if (service.outData.high_voltage > 300 and service.outData.high_voltage < 540) {
		U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
		min_ARR = (div_f / ((U_phase_max) * 5)) * 22; // 5/22 = 50/220
		if(min_ARR <= ARR_ASIN) min_ARR = ARR_ASIN;
	} else {
		U_phase_max = 220;
		min_ARR = ARR_ASIN;
	}

	U_phase = ((((service.outData.high_voltage / 20) * Km) / 141) * 112) / 100; // 31 = 620 / 20; 141 = sqrt(2) * 100; 115 = добавочный
	Km = offset + ( (Kp * (div_f / TIM3->ARR) / service.outData.high_voltage + 1 ) * 4 )/ 3;

	if (TIM3->ARR <= uint32_t(min_ARR + 5)) {
		unload = true;
		error = 0;
	}

	if (Kp > 12000) {
		Kp = 12000;
	}

	if (TIM3->ARR <= min_ARR) {
//		if (U_phase - U_phase_max > 10) {
//			Kp--;
//		} else {
//			if(adc.current() < 120 and (U_phase_max - U_phase > 10))
//			Kp++;
//		}

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

	if (TIM3->ARR > uint32_t(min_ARR + 5)) {
		if (adc.current() > 75) {
			if (Kp >= 6000) {
				Kp--;
			}
		}
	}

#endif
} else if(motor == SYNCHRON) {

//	service.outData.high_voltage = 550;

				if (service.outData.high_voltage > 300 and service.outData.high_voltage < 540) {
					U_phase_max = ((((service.outData.high_voltage / 20) * 980) / 141) * 115) / 100;
					min_ARR = ((div_f / (U_phase_max)) * 50) / 70; // 70/53 = 280/212
					if(min_ARR < value_ARR) min_ARR = value_ARR;
				} else {
					min_ARR = value_ARR;
					U_phase_max = 212;
				}

				U_phase = ((((service.outData.high_voltage / 20) * Km) / 141) * 115) / 100; // 31 = 620 / 20; 141 = sqrt(2) * 100; 115 = добавочный
//				U_phase += (U_phase_max - U_phase) * 10 / 50;
				Km = offset + Kp * (div_f / TIM3->ARR) / (service.outData.high_voltage + 1);

				if (TIM3->ARR <= uint32_t(min_ARR + 5)) {
					unload = false;
					error = 0;
				}


//				if(TIM3->ARR <= min_ARR) {
//					if ((U_phase > U_phase_max)) {
//						if((U_phase - U_phase_max) > 8)
//							Kp--;
//					} else {
//						if ((U_phase_max - U_phase > 8))
//							Kp++;
//					}
//
//					if (adc.current() > 180) {
//						if (Kp > 1250) {
//							Kp -= 4;
//						}
//					}
//				}
				if (adc.current() < 35) {
					if (Kp < 2200) {
						Kp++;
					}
				}
				if (TIM3->ARR > uint32_t(min_ARR + 5)) {
					if (adc.current() > 110) {
						if(Kp > 1250) {
							Kp--;
						}
					}
				}

				if (Kp >= 2200) {
					Kp = 2200;
				}

} //else if(motor == SYNCHRON) {

			if (Km >= 990) {
				Km = 990;
			}

			if (timer.done() and TIM3->ARR != min_ARR) {
				timer.stop();
				timer.start(time);

				if(motor == ASYNCHRON) {

					if (TIM3->ARR != min_ARR) {
						if (TIM3->ARR > uint16_t(4000)) {
							TIM3->ARR -= uint16_t(40);
						} else if (TIM3->ARR > min_ARR) {
							TIM3->ARR -= uint16_t(10);
						} else {
							TIM3->ARR++;
						}
					}

				} else if(motor == SYNCHRON) {
							if(TIM3->ARR != min_ARR) {
								if(TIM3->ARR > min_ARR) {
									if(TIM3->ARR > uint16_t(624)) {
										if(TIM3->ARR > uint16_t(1500)) {
											TIM3->ARR -= uint16_t(32);
										} else {

											TIM3->ARR -= uint16_t(3);
										}
									} else {
										TIM3->ARR-= uint16_t(1);
									}
								} else {
									TIM3->ARR++;
								}

								if(TIM3->ARR > uint16_t(624)) {
									time = 2;
								} else if (TIM3->ARR >= uint16_t(558)) {
									time = 5;
								} else if (TIM3->ARR < uint16_t(558)) {
									time = 7;
								}

							}
				} // else if(motor == SYNCHRON) {
			}
			break;
		} // switch
	} //void operator() (){

	void pusk() {

		if(motor == ASYNCHRON) {
#ifdef CONDITIONER
				frequency = 5;
				Kp = 2500;
				time = 5;
				offset = 10;
#else
				Kp = 6000;
				time = 3;
				offset = 30;
#endif

		} else if(motor == SYNCHRON) {
				frequency = 5;
				Kp = 1140;
				time = 2;
				offset = 40;
		}
		Km = 5;
		TIM3->ARR = (div_f / (frequency)) - 1;

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
		service.outData.error.phase_break = false;
		service.outData.error.HV = false;

		led_red = false;
		led_green  = false;
		blink.stop();
		if(motor == SYNCHRON)
			unload = true;
#ifdef CONDITIONER
//		work.start(600'000);
#else
		work.start(600'000);
#endif
	}

	void stop() {

		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

		HAL_TIM_Base_Stop_IT(&htim3);
		timer.stop();
		contactor.stop();

		k = 0;
		m = 12;
		n = 24;

		state = State::wait;
		adc.measure_offset();

		work.stop();

	}

	void alarm() {
		if((not Start or timer_stop.done()) or not contactor.is_on()
				      or service.outData.error.overheat_fc /*or service.outData.error.overheat_c*/ or service.outData.error.HV_low
					  or service.outData.error.voltage_board_low or service.outData.error.voltage_board_high
		  )
		{
			if(not Start and not timer_stop.isCount()) {
//				timer_stop.start(1000);
				stop();
//				timer_stop.stop();
//				if (motor == SYNCHRON) {
//					unload = true;
//					clump_timer.start(15000);
//				}
			}

			if(timer_stop.done() and not Start) {
				stop();
				timer_stop.stop();
				if (motor == SYNCHRON) {
					unload = true;
					clump_timer.start(15000);
				}
			}

			if(not contactor.is_on()
				     or service.outData.error.overheat_fc /*or service.outData.error.overheat_c*/
				     or service.outData.error.HV_low or service.outData.error.voltage_board_low or service.outData.error.voltage_board_high) {
				stop();
				timer_stop.stop();
				rerun.start(5000);
				led_red = true;
				if (motor == SYNCHRON) {
					unload = true;
					clump_timer.start(15000);
				}
			}

		}

		if (work.done() and state == starting) {
			stop();
			if (motor == SYNCHRON) {
				unload = true;
				clump_timer.start(15000);
			}
			rest.start(240'000);
			blink.start(300);
		}
//if(motor == SYNCHRON) {
//
//	if (TIM3->ARR >= (min_ARR + 5)) {
//		if (adc.is_error()) {
//			adc.reset_error();
//			if(error_F++ >= 2) {
//				phase = true;
//				error_F = 0;
//				error++;
//			}
//			led_red = true;
//			stop();
//			service.outData.error.phase_break = true;
//			rerun.start(5000);
//		}
//	}
//
//}

		if(adc.is_error_HV()) {
			adc.reset_error_HV();
//			led_red = true;
			stop();
			service.outData.error.HV = true;
			rerun.start(5000);
			if (motor == SYNCHRON) {
				unload = true;
				clump_timer.start(15000);
			}
		}

		if(adc.is_over_s() and not service.outData.error.current_S) {
			adc.reset_over_s();
			led_red = true;
			stop();
			service.outData.error.current_S = true;
			rerun.start(5000);
			if (motor == SYNCHRON) {
				unload = true;
				clump_timer.start(15000);
			}
		}

		if(adc.is_over_a() and not service.outData.error.current_A) {
			adc.reset_over_a();
			led_red = true;
			stop();
			service.outData.error.current_A = true;
			rerun.start(5000);
			if (motor == SYNCHRON) {
				unload = true;
				clump_timer.start(15000);
			}
		}

		if(adc.is_over_c() and not service.outData.error.current_C) {
			adc.reset_over_c();
			led_red = true;
			stop();
			service.outData.error.current_C = true;
			rerun.start(5000);
			if (motor == SYNCHRON) {
				unload = true;
				clump_timer.start(15000);
			}
		}

		adc.reset_measure();
	}

};

Interrupt period_callback;
//Interrupt adc_comparator_callback;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
	if(htim->Instance == TIM3) //check if the interrupt comes from ACD2
	{
		period_callback.interrupt();
	}
}

//void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc){
//	if(hadc->Instance == ADC2) //check if the interrupt comes from ACD2
//	{
//		adc_comparator_callback.interrupt();
//	}
//}

